// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "trotbot_status_led/ws2812_timing.hpp"

struct gpiod_chip;
struct gpiod_line;

namespace trotbot_status_led
{

class Ws2812Gpiod
{
public:
  using TimingNs = Ws2812TimingNs;

  Ws2812Gpiod();
  ~Ws2812Gpiod();

  Ws2812Gpiod(const Ws2812Gpiod &) = delete;
  Ws2812Gpiod & operator=(const Ws2812Gpiod &) = delete;

  bool Init(const std::string & chip_name, unsigned int line_offset, const Ws2812TimingNs & timing);
  void Shutdown();

  /** RGB per pixel, 8-bit; internally converted to GRB for WS2812. Size must equal led_count. */
  bool Show(const std::vector<uint8_t> & rgb24, unsigned int led_count);

  bool Ok() const { return line_ != nullptr; }

private:
  void SendReset();
  void SendByte(uint8_t byte);

  struct gpiod_chip * chip_{nullptr};
  struct gpiod_line * line_{nullptr};
  Ws2812TimingNs timing_{};
};

}  // namespace trotbot_status_led
