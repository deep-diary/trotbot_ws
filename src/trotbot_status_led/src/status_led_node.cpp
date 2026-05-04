// SPDX-License-Identifier: Apache-2.0
#include "trotbot_status_led/ws2812_gpiod.hpp"
#include "trotbot_status_led/ws2812_rockchip_mmap.hpp"
#include "trotbot_status_led/ws2812_spidev.hpp"
#include "trotbot_status_led/ws2812_timing.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#if defined(TROTBOT_HAS_GPIOD_MUX) && TROTBOT_HAS_GPIOD_MUX
#include <gpiod.h>
#endif

#include <cerrno>
#include <cstring>
#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <memory>
#include <string>
#include <vector>

namespace trotbot_status_led
{

namespace
{

enum class GateSpec
{
  Any,
  True,
  False
};

struct MapRule
{
  std::string state;
  GateSpec gate{GateSpec::Any};
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
  double breath_period_s{0.0};
};

struct MapData
{
  uint8_t def_r{32};
  uint8_t def_g{32};
  uint8_t def_b{32};
  std::vector<MapRule> rules;
};

GateSpec ParseGate(const YAML::Node & n)
{
  if (!n || n.IsNull()) {
    return GateSpec::Any;
  }
  try {
    const bool b = n.as<bool>();
    return b ? GateSpec::True : GateSpec::False;
  } catch (const YAML::Exception &) {
  }
  if (n.IsScalar()) {
    const std::string s = n.as<std::string>();
    if (s == "any" || s == "Any") {
      return GateSpec::Any;
    }
  }
  return GateSpec::Any;
}

bool LoadMapFile(const std::string & path, MapData * out, std::string * err)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (root["default_rgb"]) {
      const auto d = root["default_rgb"];
      if (d.IsSequence() && d.size() >= 3) {
        out->def_r = static_cast<uint8_t>(d[0].as<int>() & 0xFF);
        out->def_g = static_cast<uint8_t>(d[1].as<int>() & 0xFF);
        out->def_b = static_cast<uint8_t>(d[2].as<int>() & 0xFF);
      }
    }
    const YAML::Node rules = root["rules"];
    if (!rules || !rules.IsSequence()) {
      *err = "map file missing 'rules' sequence";
      return false;
    }
    for (const auto & item : rules) {
      MapRule rule;
      if (!item["state"]) {
        continue;
      }
      rule.state = item["state"].as<std::string>();
      rule.gate = ParseGate(item["gate"]);
      if (item["rgb"] && item["rgb"].IsSequence() && item["rgb"].size() >= 3) {
        rule.r = static_cast<uint8_t>(item["rgb"][0].as<int>() & 0xFF);
        rule.g = static_cast<uint8_t>(item["rgb"][1].as<int>() & 0xFF);
        rule.b = static_cast<uint8_t>(item["rgb"][2].as<int>() & 0xFF);
      }
      if (item["breath_period_s"]) {
        rule.breath_period_s = item["breath_period_s"].as<double>();
        if (rule.breath_period_s < 0.0) {
          rule.breath_period_s = 0.0;
        }
      }
      out->rules.push_back(rule);
    }
    return true;
  } catch (const std::exception & e) {
    *err = e.what();
    return false;
  }
}

bool MatchRule(const MapRule & rule, const std::string & st, bool gate_open)
{
  if (rule.state != st) {
    return false;
  }
  switch (rule.gate) {
    case GateSpec::Any:
      return true;
    case GateSpec::True:
      return gate_open;
    case GateSpec::False:
      return !gate_open;
  }
  return false;
}

void LookupRgb(
  const MapData & map,
  const std::string & st,
  bool gate_open,
  double time_s,
  uint8_t * r,
  uint8_t * g,
  uint8_t * b)
{
  for (const auto & rule : map.rules) {
    if (!MatchRule(rule, st, gate_open)) {
      continue;
    }
    double scale = 1.0;
    if (rule.breath_period_s > 1e-6) {
      const double ang =
        6.283185307179586476925286766559 * (time_s / rule.breath_period_s);
      scale = 0.35 + 0.65 * (std::sin(ang) * 0.5 + 0.5);
    }
    *r = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.r) * scale + 0.5)));
    *g = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.g) * scale + 0.5)));
    *b = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.b) * scale + 0.5)));
    return;
  }
  *r = map.def_r;
  *g = map.def_g;
  *b = map.def_b;
}

std::string ResolveMapPath(const std::string & param_path)
{
  if (param_path.empty()) {
    return {};
  }
  if (param_path[0] == '/') {
    return param_path;
  }
  const std::string share = ament_index_cpp::get_package_share_directory("trotbot_status_led");
  return share + "/" + param_path;
}

}  // namespace

class StatusLedNode : public rclcpp::Node
{
public:
  StatusLedNode()
  : rclcpp::Node("status_led_node")
  {
    gpiochip_ = this->declare_parameter<std::string>("gpiochip", "gpiochip0");
    gpio_line_offset_ = this->declare_parameter<int>("gpio_line_offset", 0);
    led_count_ = this->declare_parameter<int>("led_count", 16);
    global_brightness_ = this->declare_parameter<double>("global_brightness", 0.35);
    state_map_file_ = this->declare_parameter<std::string>("state_map_file", "config/status_led_map.yaml");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/power_sequence/state");
    gate_topic_ = this->declare_parameter<std::string>("gate_topic", "/power_sequence/gate_open");
    refresh_hz_ = this->declare_parameter<double>("refresh_hz", 50.0);
    simulate_hardware_ = this->declare_parameter<bool>("simulate_hardware", false);
    mlock_all_ = this->declare_parameter<bool>("mlock_all", false);
    realtime_sched_priority_ = this->declare_parameter<int>("realtime_sched_priority", -1);

    Ws2812TimingNs timing;
    timing.t0h_ns = static_cast<uint32_t>(this->declare_parameter<int>("timing_t0h_ns", 350));
    timing.t0l_ns = static_cast<uint32_t>(this->declare_parameter<int>("timing_t0l_ns", 800));
    timing.t1h_ns = static_cast<uint32_t>(this->declare_parameter<int>("timing_t1h_ns", 700));
    timing.t1l_ns = static_cast<uint32_t>(this->declare_parameter<int>("timing_t1l_ns", 600));
    timing.reset_us = static_cast<uint32_t>(this->declare_parameter<int>("timing_reset_us", 300));
    {
      const int lead_us = this->declare_parameter<int>("timing_lead_in_reset_us", 400);
      timing.lead_in_reset_us = static_cast<uint32_t>(lead_us > 0 ? lead_us : 0);
    }
    {
      const int pre_us = this->declare_parameter<int>("timing_pre_frame_idle_us", 50);
      timing.pre_frame_idle_us = static_cast<uint32_t>(pre_us > 0 ? pre_us : 0);
    }
    timing_ = timing;

    ws2812_backend_ = this->declare_parameter<std::string>("ws2812_backend", "spidev");
    mmap_gpio_phys_base_ = this->declare_parameter<int64_t>("mmap_gpio_phys_base", 0xFEC40000LL);
    if (mmap_gpio_phys_base_ < 0) {
      mmap_gpio_phys_base_ = 0xFEC40000LL;
    }

    if (global_brightness_ < 0.0) {
      global_brightness_ = 0.0;
    } else if (global_brightness_ > 1.0) {
      global_brightness_ = 1.0;
    }

    debug_fixed_color_ = this->declare_parameter<bool>("debug_fixed_color", false);
    {
      const int dr = this->declare_parameter<int>("debug_fixed_r", 0);
      const int dg = this->declare_parameter<int>("debug_fixed_g", 0);
      const int db = this->declare_parameter<int>("debug_fixed_b", 0);
      debug_fixed_r_ = static_cast<uint8_t>(std::max(0, std::min(255, dr)));
      debug_fixed_g_ = static_cast<uint8_t>(std::max(0, std::min(255, dg)));
      debug_fixed_b_ = static_cast<uint8_t>(std::max(0, std::min(255, db)));
    }

    const std::string map_path = ResolveMapPath(state_map_file_);
    std::string err;
    if (map_path.empty() || !LoadMapFile(map_path, &map_, &err)) {
      if (!debug_fixed_color_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load status map '%s': %s", map_path.c_str(), err.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "status map 未加载 (%s)，但 debug_fixed_color=true，仅用固定 RGB 发灯",
          err.c_str());
      }
      map_loaded_ = false;
    } else {
      map_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Loaded status map from %s (%zu rules)", map_path.c_str(), map_.rules.size());
    }

    if (debug_fixed_color_) {
      RCLCPP_INFO(
        this->get_logger(),
        "debug_fixed_color：忽略灯语映射，全环固定 RGB=(%u,%u,%u) × global_brightness=%.3f",
        debug_fixed_r_, debug_fixed_g_, debug_fixed_b_, global_brightness_);
    }

    hardware_ok_ = false;
    use_gpio_mmap_ = false;
    use_spidev_ = false;
    if (!simulate_hardware_) {
      if (ws2812_backend_ == "rockchip_mmap") {
        if (gpio_line_offset_ < 0 || gpio_line_offset_ > 31) {
          RCLCPP_ERROR(
            this->get_logger(),
            "rockchip_mmap 需要 gpio_line_offset 在 0..31（line 位掩码），当前=%d", gpio_line_offset_);
        } else {
#if defined(TROTBOT_HAS_GPIOD_MUX) && TROTBOT_HAS_GPIOD_MUX
          {
            struct gpiod_chip * const mux_chip = gpiod_chip_open_by_name(gpiochip_.c_str());
            if (!mux_chip) {
              RCLCPP_WARN(
                this->get_logger(),
                "gpiod: 无法打开 %s，仍将 mmap（Pad 可能未切到 GPIO）",
                gpiochip_.c_str());
            } else {
              struct gpiod_line * const mux_line =
                gpiod_chip_get_line(mux_chip, static_cast<unsigned int>(gpio_line_offset_));
              if (!mux_line) {
                RCLCPP_WARN(
                  this->get_logger(), "gpiod: chip %s 无 offset %d", gpiochip_.c_str(), gpio_line_offset_);
              } else if (gpiod_line_request_output(mux_line, "trotbot_status_led_mux", 0) < 0) {
                RCLCPP_WARN(
                  this->get_logger(),
                  "gpiod: request_output(%s,%d) 失败（busy？）。chmod /dev/%s 或释放占用",
                  gpiochip_.c_str(), gpio_line_offset_, gpiochip_.c_str());
              } else {
                gpiod_line_release(mux_line);
                RCLCPP_INFO(
                  this->get_logger(),
                  "gpiod: 已完成 Pad→GPIO 配置并立即释放 line（避免与 mmap 争用同一 DR 寄存器）");
              }
              gpiod_chip_close(mux_chip);
            }
          }
#endif
          const uint32_t pin_mask = 1U << static_cast<unsigned int>(gpio_line_offset_);
          if (mmap_.Init(static_cast<uint64_t>(mmap_gpio_phys_base_), pin_mask, timing_)) {
            hardware_ok_ = true;
            use_gpio_mmap_ = true;
            RCLCPP_INFO(
              this->get_logger(),
              "WS2812 rockchip_mmap ok: phys=0x%llx line=%d (mask=0x%x) leds=%d",
              static_cast<unsigned long long>(static_cast<uint64_t>(mmap_gpio_phys_base_)),
              gpio_line_offset_, pin_mask, led_count_);
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "rockchip_mmap 打开 /dev/mem 或 mmap 失败（常因无 root）。"
              " 请用 sudo 运行本节点，或改 ws2812_backend:=gpiod 并 chmod /dev/gpiochip*（仅 gpiod 易发白）。"
              " 见 README。");
            RCLCPP_ERROR(
              this->get_logger(),
              "硬件未就绪：本进程将不再刷新 WS2812。"
              " 灯环会长时间保持「上一次成功点亮」的最后一帧（常见表现为脚本停在最后一步的状态，例如 Precheck）——并非订阅失效。");
          }
        }
      } else if (ws2812_backend_ == "gpiod") {
        if (gpiod_.Init(gpiochip_, static_cast<unsigned int>(gpio_line_offset_), timing_)) {
          hardware_ok_ = true;
          RCLCPP_INFO(
            this->get_logger(), "WS2812 gpiod ok: chip=%s offset=%d leds=%d",
            gpiochip_.c_str(), gpio_line_offset_, led_count_);
        } else {
#if defined(TROTBOT_STATUS_LED_GPIO_STUB) && TROTBOT_STATUS_LED_GPIO_STUB
          RCLCPP_ERROR(
            this->get_logger(),
            "WS2812 为 STUB 构建（未链 libgpiod），gpiod 后端无法发码。"
            " 可改 ws2812_backend:=rockchip_mmap 并用 sudo 跑（不依赖 libgpiod）或安装 libgpiod-dev 后重编。");
#else
          RCLCPP_WARN(
            this->get_logger(),
            "GPIO/WS2812 gpiod init 失败 (chip=%s line=%d)。可试 rockchip_mmap + sudo。",
            gpiochip_.c_str(), gpio_line_offset_);
#endif
        }
      } else if (ws2812_backend_ == "spidev") {
        const std::string spidev_path = this->declare_parameter<std::string>("spidev_path", "/dev/spidev0.0");
        const int spd_hz = this->declare_parameter<int>("spidev_max_speed_hz", 6400000);
        const int rst_bytes = this->declare_parameter<int>("spidev_reset_trailer_bytes", 120);
        const int pre_idle_us = this->declare_parameter<int>("spidev_pre_write_idle_us", 0);
        const int lead_zeros = this->declare_parameter<int>("spidev_leading_spi_zero_bytes", 0);
        const int b0 = this->declare_parameter<int>("spidev_bit0_byte", -1);
        const int b1 = this->declare_parameter<int>("spidev_bit1_byte", -1);
        const uint8_t byte0 =
          (b0 >= 0 && b0 <= 255) ? static_cast<uint8_t>(b0) : static_cast<uint8_t>(0xC0);
        const uint8_t byte1 =
          (b1 >= 0 && b1 <= 255) ? static_cast<uint8_t>(b1) : static_cast<uint8_t>(0xF8);
        const uint32_t hz_u =
          spd_hz > 0 ? static_cast<uint32_t>(spd_hz) : 6400000U;
        const unsigned trailer =
          rst_bytes > 0 ? static_cast<unsigned>(rst_bytes) : 120U;
        const uint32_t idle_u =
          pre_idle_us > 0 ? static_cast<uint32_t>(pre_idle_us) : 0U;
        const unsigned lead_z =
          lead_zeros > 0 ? static_cast<unsigned>(lead_zeros) : 0U;
        if (spidev_.Init(spidev_path, hz_u, trailer, byte0, byte1, idle_u, lead_z)) {
          hardware_ok_ = true;
          use_spidev_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "WS2812 spidev ok: path=%s speed_hz=%u reset_trailer_bytes=%u enc=(0x%02x,0x%02x) "
            "pre_write_idle_us=%u leading_spi_zero_bytes=%u leds=%d",
            spidev_path.c_str(), hz_u, trailer, byte0, byte1, idle_u, lead_z, led_count_);
          if (!spidev_.SpidevIoctlOk()) {
            RCLCPP_WARN(
              this->get_logger(),
              "SPI 节点未响应标准 spidev ioctl（常见于鲁班猫 rockchip,spidev → /dev/rkspi-dev*）。"
              " 将以 write() 发码；实际波特率由 overlay 中 spi-max-frequency 决定，若颜色不对请对照 docs/SPI_WS2812_RK3588.md 调整编码或时钟。");
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "WS2812 spidev 打开失败 path=%s（检查设备树是否启用 SPI+spidev、节点是否存在、用户是否在 dialout 组）。"
            " 见 docs/SPI_WS2812_RK3588.md",
            spidev_path.c_str());
        }
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "未知 ws2812_backend=%s（rockchip_mmap / gpiod / spidev）", ws2812_backend_.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "simulate_hardware=true: no GPIO writes.");
      hardware_ok_ = false;
    }

    if (hardware_ok_ && !simulate_hardware_) {
      if (mlock_all_) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
          RCLCPP_INFO(this->get_logger(), "mlockall: 已将进程内存锁定（减轻 WS2812 时序被换页打断）");
        } else {
          RCLCPP_WARN(
            this->get_logger(), "mlockall 失败 (%s)，可忽略或检查 ulimit", strerror(errno));
        }
      }
      if (realtime_sched_priority_ >= 1 && realtime_sched_priority_ <= 99) {
        struct sched_param sp {};
        sp.sched_priority = realtime_sched_priority_;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0) {
          RCLCPP_INFO(
            this->get_logger(), "SCHED_FIFO 已启用，priority=%d（需 root / 能力）", realtime_sched_priority_);
        } else {
          RCLCPP_WARN(
            this->get_logger(), "SCHED_FIFO 设置失败 (%s)，继续普通调度", strerror(errno));
        }
      }
    }

    rclcpp::QoS gate_qos(rclcpp::KeepLast(10));
    gate_qos.transient_local().reliable();
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(10),
      std::bind(&StatusLedNode::OnState, this, std::placeholders::_1));
    gate_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      gate_topic_, gate_qos,
      std::bind(&StatusLedNode::OnGate, this, std::placeholders::_1));

    const double hz = refresh_hz_ > 0.1 ? refresh_hz_ : 50.0;
    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StatusLedNode::OnTimer, this));

    phase_time_s_ = 0.0;
  }

  ~StatusLedNode() override
  {
    mmap_.Shutdown();
    gpiod_.Shutdown();
    spidev_.Shutdown();
  }

private:
  void OnState(const std_msgs::msg::String::SharedPtr msg) { last_state_ = msg->data; }

  void OnGate(const std_msgs::msg::Bool::SharedPtr msg) { last_gate_ = msg->data; }

  void OnTimer()
  {
    if (!debug_fixed_color_ && !map_loaded_) {
      return;
    }
    const double dt = 1.0 / refresh_hz_;
    phase_time_s_ += dt;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    if (debug_fixed_color_) {
      r = debug_fixed_r_;
      g = debug_fixed_g_;
      b = debug_fixed_b_;
    } else {
      LookupRgb(map_, last_state_, last_gate_, phase_time_s_, &r, &g, &b);
    }

    r = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(r) * global_brightness_ + 0.5)));
    g = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(g) * global_brightness_ + 0.5)));
    b = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(b) * global_brightness_ + 0.5)));

    std::vector<uint8_t> buf(static_cast<size_t>(led_count_) * 3U, 0);
    for (int i = 0; i < led_count_; ++i) {
      buf[static_cast<size_t>(i) * 3U + 0] = r;
      buf[static_cast<size_t>(i) * 3U + 1] = g;
      buf[static_cast<size_t>(i) * 3U + 2] = b;
    }

    if (hardware_ok_ && !simulate_hardware_) {
      if (use_gpio_mmap_) {
        (void)mmap_.Show(buf, static_cast<unsigned int>(led_count_));
      } else if (use_spidev_) {
        (void)spidev_.Show(buf, static_cast<unsigned int>(led_count_));
      } else {
        (void)gpiod_.Show(buf, static_cast<unsigned int>(led_count_));
      }
    }
  }

  Ws2812RockchipMmap mmap_;
  Ws2812Gpiod gpiod_;
  Ws2812Spidev spidev_;
  Ws2812TimingNs timing_{};

  std::string ws2812_backend_{"spidev"};
  int64_t mmap_gpio_phys_base_{0xFEC40000LL};

  std::string gpiochip_;
  int gpio_line_offset_{0};
  int led_count_{16};
  double global_brightness_{0.35};
  std::string state_map_file_;
  std::string state_topic_;
  std::string gate_topic_;
  double refresh_hz_{50.0};
  bool simulate_hardware_{false};
  bool debug_fixed_color_{false};
  uint8_t debug_fixed_r_{0};
  uint8_t debug_fixed_g_{0};
  uint8_t debug_fixed_b_{0};
  bool mlock_all_{false};
  int realtime_sched_priority_{-1};

  MapData map_{};
  bool map_loaded_{false};

  std::string last_state_{"Idle"};
  bool last_gate_{false};

  bool hardware_ok_{false};
  bool use_gpio_mmap_{false};
  bool use_spidev_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double phase_time_s_{0.0};
};

}  // namespace trotbot_status_led

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trotbot_status_led::StatusLedNode>());
  rclcpp::shutdown();
  return 0;
}
