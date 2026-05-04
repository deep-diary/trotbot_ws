// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace trotbot_status_led
{

/**
 * 通过 Linux spidev 将 WS2812 NRZ 波形编码为 SPI 字节流（常见：6.4MHz、每 WS2812 bit 对应 1 个 SPI 字节）。
 * 无需 GPIO bitbang；通常不需 root（设备节点权限即可）。
 */
class Ws2812Spidev
{
public:
  Ws2812Spidev();
  ~Ws2812Spidev();

  Ws2812Spidev(const Ws2812Spidev &) = delete;
  Ws2812Spidev & operator=(const Ws2812Spidev &) = delete;

  /**
   * @param reset_trailer_bytes 帧尾连续 0x00 字节数，形成 WS2812 reset 低电平间隙（与 SPI 时钟有关）。
   * @param bit0_byte SPI 图案表示 WS2812「0」码（默认 0xC0 @ 6.4MHz）
   * @param bit1_byte SPI 图案表示 WS2812「1」码（默认 0xF8 @ 6.4MHz）
   * @param pre_write_idle_us 写 SPI 前空闲（微秒），拉长首字节前的低电平，减轻首颗灯 GR 误判闪绿
   * @param leading_spi_zero_bytes 在 WS2812 编码载荷前追加若干 SPI 裸字节 0x00（实验性；部分板可用）
   */
  bool Init(
    const std::string & dev_path, uint32_t max_speed_hz, unsigned reset_trailer_bytes,
    uint8_t bit0_byte, uint8_t bit1_byte, uint32_t pre_write_idle_us = 0,
    unsigned leading_spi_zero_bytes = 0);

  void Shutdown();

  /** RGB 每像素 8bit；内部按 GRB 顺序编码为 WS2812。 */
  bool Show(const std::vector<uint8_t> & rgb24, unsigned int led_count);

  bool Ok() const { return fd_ >= 0; }

  /** true：已对字符设备设置 MODE/BITS/SPEED（Linux spidev）。false：仅打开 fd（如鲁班猫 rockchip,spidev → rkspi-dev）。 */
  bool SpidevIoctlOk() const { return spidev_ioctl_ok_; }

private:
  int fd_{-1};
  bool spidev_ioctl_ok_{false};
  uint32_t max_speed_hz_{6400000};
  unsigned reset_trailer_bytes_{120};
  uint8_t bit0_byte_{0xC0};
  uint8_t bit1_byte_{0xF8};
  uint32_t pre_write_idle_us_{0};
  unsigned leading_spi_zero_bytes_{0};
};

}  // namespace trotbot_status_led
