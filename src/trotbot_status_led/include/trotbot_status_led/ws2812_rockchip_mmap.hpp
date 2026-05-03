// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "trotbot_status_led/ws2812_timing.hpp"

namespace trotbot_status_led
{

/**
 * 通过 /dev/mem mmap Rockchip GPIO bank，按 **GPIO 控制器 V2** 协议写 DR（RK3588 等）。
 * V2 不可使用简单 *DR|=bit：须按 linux/drivers/gpio/gpio-rockchip.c 的 rockchip_gpio_writel_bit 写掩码。
 */
class Ws2812RockchipMmap
{
public:
  Ws2812RockchipMmap();
  ~Ws2812RockchipMmap();

  Ws2812RockchipMmap(const Ws2812RockchipMmap &) = delete;
  Ws2812RockchipMmap & operator=(const Ws2812RockchipMmap &) = delete;

  /** phys_base：GPIO bank 物理基址；pin_mask：单 bit，如 (1<<17) */
  bool Init(uint64_t phys_base, uint32_t pin_mask, const Ws2812TimingNs & timing);
  void Shutdown();

  bool Show(const std::vector<uint8_t> & rgb24, unsigned int led_count);

  bool Ok() const { return bank_base_u32_ != nullptr; }

private:
  void SendReset();
  void SendByte(uint8_t byte);
  void SetDr(bool high);

  void * mapped_{nullptr};
  size_t map_length_{0};
  /** 映射后的 bank 基址（对应寄存器空间 offset 0） */
  volatile uint32_t * bank_base_u32_{nullptr};
  unsigned pin_bit_{0};
  Ws2812TimingNs timing_{};
  int mem_fd_{-1};
};

}  // namespace trotbot_status_led
