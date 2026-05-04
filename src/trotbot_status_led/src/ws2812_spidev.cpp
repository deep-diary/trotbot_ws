// SPDX-License-Identifier: Apache-2.0
#include "trotbot_status_led/ws2812_spidev.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <thread>
#include <vector>

namespace trotbot_status_led
{

Ws2812Spidev::Ws2812Spidev() = default;

Ws2812Spidev::~Ws2812Spidev()
{
  Shutdown();
}

bool Ws2812Spidev::Init(
  const std::string & dev_path, const uint32_t max_speed_hz, const unsigned reset_trailer_bytes,
  const uint8_t bit0_byte, const uint8_t bit1_byte, const uint32_t pre_write_idle_us,
  const unsigned leading_spi_zero_bytes)
{
  Shutdown();
  if (dev_path.empty() || max_speed_hz < 100000U) {
    return false;
  }
  max_speed_hz_ = max_speed_hz;
  reset_trailer_bytes_ = reset_trailer_bytes > 4000U ? 4000U : reset_trailer_bytes;
  bit0_byte_ = bit0_byte;
  bit1_byte_ = bit1_byte;
  pre_write_idle_us_ = pre_write_idle_us;
  leading_spi_zero_bytes_ = leading_spi_zero_bytes > 512U ? 512U : leading_spi_zero_bytes;

  fd_ = open(dev_path.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }

  spidev_ioctl_ok_ = false;
  uint8_t mode = SPI_MODE_0;
  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
    /* 鲁班猫 SPI overlay 常为 compatible=rockchip,spidev → misc rkspi-dev，非标准 spidev ioctl */
    return true;
  }
  uint8_t bits = 8;
  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
    return true;
  }
  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_hz_) < 0) {
    return true;
  }
  spidev_ioctl_ok_ = true;
  return true;
}

void Ws2812Spidev::Shutdown()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  spidev_ioctl_ok_ = false;
}

bool Ws2812Spidev::Show(const std::vector<uint8_t> & rgb24, const unsigned int led_count)
{
  if (fd_ < 0) {
    return false;
  }
  const size_t expected = static_cast<size_t>(led_count) * 3U;
  if (rgb24.size() < expected) {
    return false;
  }

  if (pre_write_idle_us_ > 0U) {
    std::this_thread::sleep_for(std::chrono::microseconds(pre_write_idle_us_));
  }

  // 每 WS2812 数据 bit → 1 个 SPI 字节（8 个 SCLK 沿）；reset 用 0x00 拉长低电平
  const size_t payload_bits = static_cast<size_t>(led_count) * 3U * 8U;
  std::vector<uint8_t> wire;
  wire.reserve(leading_spi_zero_bytes_ + payload_bits + reset_trailer_bytes_);

  for (unsigned i = 0; i < leading_spi_zero_bytes_; ++i) {
    wire.push_back(0U);
  }

  for (unsigned int li = 0; li < led_count; ++li) {
    const size_t base = static_cast<size_t>(li) * 3U;
    const uint8_t px_r = rgb24[base + 0];
    const uint8_t px_g = rgb24[base + 1];
    const uint8_t px_b = rgb24[base + 2];
    const uint8_t grb[3] = {px_g, px_r, px_b};
    for (unsigned c = 0; c < 3U; ++c) {
      const uint8_t ch = grb[c];
      for (int bi = 7; bi >= 0; --bi) {
        const bool one = ((ch >> bi) & 1U) != 0U;
        wire.push_back(one ? bit1_byte_ : bit0_byte_);
      }
    }
  }
  for (unsigned i = 0; i < reset_trailer_bytes_; ++i) {
    wire.push_back(0U);
  }

  const ssize_t w = write(fd_, wire.data(), wire.size());
  return w == static_cast<ssize_t>(wire.size());
}

}  // namespace trotbot_status_led
