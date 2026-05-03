// SPDX-License-Identifier: Apache-2.0
#include "trotbot_status_led/ws2812_gpiod.hpp"

#include "trotbot_status_led/ws2812_busy_delay.hpp"

#include <gpiod.h>

#include <chrono>
#include <thread>

namespace trotbot_status_led
{

Ws2812Gpiod::Ws2812Gpiod() = default;

Ws2812Gpiod::~Ws2812Gpiod()
{
  Shutdown();
}

bool Ws2812Gpiod::Init(const std::string & chip_name, unsigned int line_offset, const Ws2812TimingNs & timing)
{
  Shutdown();
  timing_ = timing;
  chip_ = gpiod_chip_open_by_name(chip_name.c_str());
  if (!chip_) {
    return false;
  }
  line_ = gpiod_chip_get_line(chip_, line_offset);
  if (!line_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }
  if (gpiod_line_request_output(line_, "trotbot_status_led", 0) < 0) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    line_ = nullptr;
    return false;
  }
  gpiod_line_set_value(line_, 0);
  return true;
}

void Ws2812Gpiod::Shutdown()
{
  if (line_) {
    gpiod_line_set_value(line_, 0);
    gpiod_line_release(line_);
    line_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
}

void Ws2812Gpiod::SendReset()
{
  if (!line_) {
    return;
  }
  gpiod_line_set_value(line_, 0);
  std::this_thread::sleep_for(std::chrono::microseconds(timing_.reset_us));
}

void Ws2812Gpiod::SendByte(uint8_t byte)
{
  if (!line_) {
    return;
  }
  for (int i = 7; i >= 0; --i) {
    const bool bit = (byte >> i) & 1;
    if (bit) {
      gpiod_line_set_value(line_, 1);
      BusyDelayNs(static_cast<int64_t>(timing_.t1h_ns));
      gpiod_line_set_value(line_, 0);
      BusyDelayNs(static_cast<int64_t>(timing_.t1l_ns));
    } else {
      gpiod_line_set_value(line_, 1);
      BusyDelayNs(static_cast<int64_t>(timing_.t0h_ns));
      gpiod_line_set_value(line_, 0);
      BusyDelayNs(static_cast<int64_t>(timing_.t0l_ns));
    }
  }
}

bool Ws2812Gpiod::Show(const std::vector<uint8_t> & rgb24, unsigned int led_count)
{
  if (!line_) {
    return false;
  }
  const size_t expected = static_cast<size_t>(led_count) * 3U;
  if (rgb24.size() < expected) {
    return false;
  }

  gpiod_line_set_value(line_, 0);
  if (timing_.lead_in_reset_us > 0U) {
    std::this_thread::sleep_for(std::chrono::microseconds(timing_.lead_in_reset_us));
  } else if (timing_.pre_frame_idle_us > 0U) {
    std::this_thread::sleep_for(std::chrono::microseconds(timing_.pre_frame_idle_us));
  }

  for (unsigned int i = 0; i < led_count; ++i) {
    const size_t base = static_cast<size_t>(i) * 3U;
    const uint8_t r = rgb24[base + 0];
    const uint8_t g = rgb24[base + 1];
    const uint8_t b = rgb24[base + 2];
    SendByte(g);
    SendByte(r);
    SendByte(b);
  }
  SendReset();
  return true;
}

}  // namespace trotbot_status_led
