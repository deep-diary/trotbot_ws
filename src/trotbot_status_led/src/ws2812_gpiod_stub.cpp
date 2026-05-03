// SPDX-License-Identifier: Apache-2.0
// 构建环境未安装 libgpiod 时的占位实现（Init 恒失败，无 GPIO 访问）。
#include "trotbot_status_led/ws2812_gpiod.hpp"

namespace trotbot_status_led
{

Ws2812Gpiod::Ws2812Gpiod() = default;

Ws2812Gpiod::~Ws2812Gpiod()
{
  Shutdown();
}

bool Ws2812Gpiod::Init(
  const std::string & /*chip_name*/, unsigned int /*line_offset*/,
  const Ws2812TimingNs & /*timing*/)
{
  return false;
}

void Ws2812Gpiod::Shutdown() {}

bool Ws2812Gpiod::Show(const std::vector<uint8_t> & /*rgb24*/, unsigned int /*led_count*/)
{
  return false;
}

}  // namespace trotbot_status_led
