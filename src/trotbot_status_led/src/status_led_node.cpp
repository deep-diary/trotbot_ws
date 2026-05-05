// SPDX-License-Identifier: Apache-2.0
#include "trotbot_status_led/ws2812_gpiod.hpp"
#include "trotbot_status_led/ws2812_rockchip_mmap.hpp"
#include "trotbot_status_led/ws2812_spidev.hpp"
#include "trotbot_status_led/ws2812_timing.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <yaml-cpp/yaml.h>

#if defined(TROTBOT_HAS_GPIOD_MUX) && TROTBOT_HAS_GPIOD_MUX
#include <gpiod.h>
#endif

#include <cctype>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <algorithm>
#include <memory>
#include <sstream>
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

/** YAML effect / 扩展：流水灯预留 Chase（当前按同色 solid 渲染）。 */
enum class MapEffect
{
  Solid,
  Breath,
  Chase
};

struct MapRule
{
  std::string state;
  GateSpec gate{GateSpec::Any};
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
  double breath_period_s{0.0};
  MapEffect effect{MapEffect::Solid};
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
      rule.effect = MapEffect::Solid;
      if (item["effect"]) {
        const std::string ef = item["effect"].as<std::string>();
        if (ef == "breath" || ef == "Breath") {
          rule.effect = MapEffect::Breath;
        } else if (ef == "chase" || ef == "Chase") {
          rule.effect = MapEffect::Chase;
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

bool FindMatchingRule(const MapData & map, const std::string & st, bool gate_open, MapRule * out)
{
  for (const auto & rule : map.rules) {
    if (!MatchRule(rule, st, gate_open)) {
      continue;
    }
    *out = rule;
    return true;
  }
  return false;
}

bool RuleNeedsBreathAnimation(const MapRule & rule)
{
  if (rule.effect == MapEffect::Breath) {
    return true;
  }
  if (rule.breath_period_s > 1e-6) {
    return true;
  }
  return false;
}

void LookupRgbFromRule(const MapRule & rule, double time_s, uint8_t * r, uint8_t * g, uint8_t * b)
{
  double scale = 1.0;
  const bool breath_like =
    (rule.effect == MapEffect::Breath || rule.breath_period_s > 1e-6) &&
    rule.effect != MapEffect::Chase;
  if (breath_like) {
    const double period = rule.breath_period_s > 1e-6 ? rule.breath_period_s : 1.2;
    const double ang =
      6.283185307179586476925286766559 * (time_s / period);
    scale = 0.35 + 0.65 * (std::sin(ang) * 0.5 + 0.5);
  }
  // Chase：预留，当前与同色 solid 一致（逐灯偏移后续实现）
  *r = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.r) * scale + 0.5)));
  *g = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.g) * scale + 0.5)));
  *b = static_cast<uint8_t>(std::min(255.0, std::floor(static_cast<double>(rule.b) * scale + 0.5)));
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
  MapRule rule;
  if (FindMatchingRule(map, st, gate_open, &rule)) {
    LookupRgbFromRule(rule, time_s, r, g, b);
    return;
  }
  *r = map.def_r;
  *g = map.def_g;
  *b = map.def_b;
}

std::string TrimWs(std::string s)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
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
    led_count_ = this->declare_parameter<int>("led_count", 8);
    global_brightness_ = this->declare_parameter<double>("global_brightness", 0.35);
    state_map_file_ = this->declare_parameter<std::string>("state_map_file", "config/status_led_map.yaml");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/power_sequence/state");
    gate_topic_ = this->declare_parameter<std::string>("gate_topic", "/power_sequence/gate_open");
    refresh_hz_ = this->declare_parameter<double>("refresh_hz", 50.0);
    refresh_on_change_only_ = this->declare_parameter<bool>("refresh_on_change_only", true);
    change_repeat_count_ = this->declare_parameter<int>("change_repeat_count", 2);
    change_repeat_interval_ms_ = this->declare_parameter<int>("change_repeat_interval_ms", 5);
    animation_refresh_hz_ = this->declare_parameter<double>("animation_refresh_hz", 40.0);
    initial_push_delay_ms_ = this->declare_parameter<int>("initial_push_delay_ms", 50);
    publish_debug_rgb_ = this->declare_parameter<bool>("publish_debug_rgb", false);
    debug_rgb_topic_ = this->declare_parameter<std::string>("debug_rgb_topic", "/status_led/debug_rgb");
    debug_meta_topic_ = this->declare_parameter<std::string>("debug_meta_topic", "/status_led/debug_meta");
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
    rclcpp::QoS state_qos(rclcpp::KeepLast(1));
    state_qos.transient_local().reliable();
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      state_topic_, state_qos,
      std::bind(&StatusLedNode::OnState, this, std::placeholders::_1));
    gate_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      gate_topic_, gate_qos,
      std::bind(&StatusLedNode::OnGate, this, std::placeholders::_1));

    if (publish_debug_rgb_) {
      debug_rgb_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(debug_rgb_topic_, 10);
      debug_meta_pub_ = this->create_publisher<std_msgs::msg::String>(debug_meta_topic_, 10);
      RCLCPP_INFO(
        this->get_logger(), "调试话题：'publish_debug_rgb'=true → '%s'（RGB）'%s'（state/gate/匹配）",
        debug_rgb_topic_.c_str(), debug_meta_topic_.c_str());
    }

    phase_time_s_ = 0.0;

    if (debug_fixed_color_) {
      const double hz = refresh_hz_ > 0.1 ? refresh_hz_ : 50.0;
      const auto period = std::chrono::duration<double>(1.0 / hz);
      legacy_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&StatusLedNode::OnDebugFixedTimer, this));
    } else if (!refresh_on_change_only_) {
      const double hz = refresh_hz_ > 0.1 ? refresh_hz_ : 50.0;
      const auto period = std::chrono::duration<double>(1.0 / hz);
      legacy_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&StatusLedNode::OnLegacyPeriodicTimer, this));
      RCLCPP_INFO(
        this->get_logger(), "WS2812 周期刷新 refresh_hz=%.2f（refresh_on_change_only:=false）", hz);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "WS2812 按变化刷新：repeat=%d interval_ms=%d animation_hz=%.2f",
        change_repeat_count_, change_repeat_interval_ms_, animation_refresh_hz_);
      const int delay_ms = std::max(0, initial_push_delay_ms_);
      startup_push_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(delay_ms > 0 ? delay_ms : 1),
        [this]() {
          if (startup_push_timer_) {
            startup_push_timer_->cancel();
            startup_push_timer_.reset();
          }
          RefreshOutputs();
        });
    }
  }

  ~StatusLedNode() override
  {
    CancelRepeatTimer();
    CancelAnimationTimer();
    if (startup_push_timer_) {
      startup_push_timer_->cancel();
    }
    if (legacy_timer_) {
      legacy_timer_->cancel();
    }
    mmap_.Shutdown();
    gpiod_.Shutdown();
    spidev_.Shutdown();
  }

private:
  void CancelRepeatTimer()
  {
    if (repeat_timer_) {
      repeat_timer_->cancel();
      repeat_timer_.reset();
    }
    repeat_remaining_ = 0;
  }

  void CancelAnimationTimer()
  {
    if (animation_timer_) {
      animation_timer_->cancel();
      animation_timer_.reset();
    }
  }

  void PublishDebugRgb(uint8_t r, uint8_t g, uint8_t b, bool map_matched)
  {
    if (!publish_debug_rgb_) {
      return;
    }
    if (!debug_rgb_pub_ || !debug_meta_pub_) {
      return;
    }
    std_msgs::msg::UInt8MultiArray rgb_msg;
    rgb_msg.data.resize(3);
    rgb_msg.data[0] = r;
    rgb_msg.data[1] = g;
    rgb_msg.data[2] = b;
    debug_rgb_pub_->publish(rgb_msg);

    std_msgs::msg::String meta;
    std::ostringstream oss;
    oss << "state=\"" << last_state_ << "\" gate_open=" << (last_gate_ ? 1 : 0)
        << " rgb_u8=" << static_cast<int>(r) << "," << static_cast<int>(g) << "," << static_cast<int>(b)
        << " global_brightness=" << global_brightness_ << " map_match=" << (map_matched ? 1 : 0)
        << " map_loaded=" << (map_loaded_ ? 1 : 0);
    meta.data = oss.str();
    debug_meta_pub_->publish(meta);
  }

  void DoRenderPushToHardware()
  {
    if (!debug_fixed_color_ && !map_loaded_) {
      return;
    }

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

    bool map_matched = false;
    if (map_loaded_ && !debug_fixed_color_) {
      MapRule dbg_rule{};
      map_matched = FindMatchingRule(map_, last_state_, last_gate_, &dbg_rule);
    }
    PublishDebugRgb(r, g, b, map_matched);

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

  void PushSolidBurst()
  {
    DoRenderPushToHardware();
    const int extra = change_repeat_count_ - 1;
    if (extra <= 0) {
      return;
    }
    repeat_remaining_ = extra;
    const int ms = std::max(1, change_repeat_interval_ms_);
    repeat_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(ms),
      [this]() {
        DoRenderPushToHardware();
        repeat_remaining_--;
        if (repeat_remaining_ <= 0) {
          CancelRepeatTimer();
        }
      });
  }

  void StartAnimationTimer()
  {
    CancelAnimationTimer();
    const double hz = animation_refresh_hz_ > 0.1 ? animation_refresh_hz_ : 40.0;
    const auto period = std::chrono::duration<double>(1.0 / hz);
    animation_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StatusLedNode::OnAnimationTick, this));
  }

  void OnAnimationTick()
  {
    const double hz = animation_refresh_hz_ > 0.1 ? animation_refresh_hz_ : 40.0;
    phase_time_s_ += 1.0 / hz;
    DoRenderPushToHardware();
  }

  void RefreshOutputs()
  {
    if (debug_fixed_color_) {
      return;
    }
    if (!map_loaded_) {
      return;
    }

    CancelRepeatTimer();
    CancelAnimationTimer();

    MapRule rule{};
    if (!FindMatchingRule(map_, last_state_, last_gate_, &rule)) {
      PushSolidBurst();
      return;
    }

    if (RuleNeedsBreathAnimation(rule)) {
      phase_time_s_ = 0.0;
      StartAnimationTimer();
      OnAnimationTick();
      return;
    }

    PushSolidBurst();
  }

  void OnLegacyPeriodicTimer()
  {
    if (!map_loaded_) {
      return;
    }
    const double hz = refresh_hz_ > 0.1 ? refresh_hz_ : 50.0;
    phase_time_s_ += 1.0 / hz;
    DoRenderPushToHardware();
  }

  void OnDebugFixedTimer() { DoRenderPushToHardware(); }

  void OnState(const std_msgs::msg::String::SharedPtr msg)
  {
    last_state_ = TrimWs(msg->data);
    if (!debug_fixed_color_ && refresh_on_change_only_) {
      RefreshOutputs();
    }
  }

  void OnGate(const std_msgs::msg::Bool::SharedPtr msg)
  {
    last_gate_ = msg->data;
    if (!debug_fixed_color_ && refresh_on_change_only_) {
      RefreshOutputs();
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
  int led_count_{8};
  double global_brightness_{0.35};
  std::string state_map_file_;
  std::string state_topic_;
  std::string gate_topic_;
  double refresh_hz_{50.0};
  bool refresh_on_change_only_{true};
  int change_repeat_count_{2};
  int change_repeat_interval_ms_{5};
  double animation_refresh_hz_{40.0};
  int initial_push_delay_ms_{50};
  bool publish_debug_rgb_{false};
  std::string debug_rgb_topic_{"/status_led/debug_rgb"};
  std::string debug_meta_topic_{"/status_led/debug_meta"};
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr debug_rgb_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_meta_pub_;
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
  rclcpp::TimerBase::SharedPtr legacy_timer_;
  rclcpp::TimerBase::SharedPtr repeat_timer_;
  rclcpp::TimerBase::SharedPtr animation_timer_;
  rclcpp::TimerBase::SharedPtr startup_push_timer_;
  int repeat_remaining_{0};

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
