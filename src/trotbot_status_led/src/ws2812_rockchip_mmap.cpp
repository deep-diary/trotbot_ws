// SPDX-License-Identifier: Apache-2.0
#include "trotbot_status_led/ws2812_rockchip_mmap.hpp"

#include "trotbot_status_led/ws2812_busy_delay.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <thread>

namespace trotbot_status_led
{

namespace
{
constexpr uintptr_t kPageMask = 4096ULL - 1U;

/** RK3588 等为 GPIO V2：port_ddr=0x08，DR 为 0x00 起双字，见 linux/drivers/gpio/gpio-rockchip.c gpio_regs_v2 */
constexpr unsigned kRockchipV2PortDrOffsetBytes = 0x00;
constexpr unsigned kRockchipV2PortDdrOffsetBytes = 0x08;

inline uintptr_t PageFloor(uint64_t phys)
{
  return static_cast<uintptr_t>(phys) & ~kPageMask;
}

inline size_t PageSpan(uint64_t phys, size_t reg_span_bytes)
{
  const uintptr_t start = PageFloor(phys);
  const uintptr_t end = static_cast<uintptr_t>(phys + reg_span_bytes + kPageMask) & ~kPageMask;
  return static_cast<size_t>(end - start);
}

/** 等价于 rockchip_gpio_writel_bit(..., value, port_dr 或 port_ddr)，仅适用于 GPIO V2 */
inline void RockchipGpioV2WriteHalf(
  volatile uint32_t * reg_pair_start, unsigned bit, bool value)
{
  const uint32_t data = value
    ? ((1U << (bit % 16)) | (1U << ((bit % 16) + 16)))
    : (1U << ((bit % 16) + 16));
  volatile uint32_t * const target = (bit >= 16) ? (reg_pair_start + 1) : reg_pair_start;
  *target = data;
}
}  // namespace

Ws2812RockchipMmap::Ws2812RockchipMmap() = default;

Ws2812RockchipMmap::~Ws2812RockchipMmap()
{
  Shutdown();
}

bool Ws2812RockchipMmap::Init(uint64_t phys_base, uint32_t pin_mask, const Ws2812TimingNs & timing)
{
  Shutdown();
  timing_ = timing;
  if (pin_mask == 0U || (pin_mask & (pin_mask - 1U)) != 0U) {
    return false;
  }
  pin_bit_ = static_cast<unsigned>(__builtin_ctz(pin_mask));

  mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd_ < 0) {
    return false;
  }

  const uintptr_t page_base = PageFloor(phys_base);
  const size_t offset_in_mapping = static_cast<size_t>(phys_base - page_base);
  map_length_ = PageSpan(phys_base, 0x100);
  mapped_ = mmap(nullptr, map_length_, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd_, static_cast<off_t>(page_base));
  if (mapped_ == MAP_FAILED) {
    mapped_ = nullptr;
    close(mem_fd_);
    mem_fd_ = -1;
    return false;
  }

  bank_base_u32_ = reinterpret_cast<volatile uint32_t *>(
    static_cast<char *>(mapped_) + offset_in_mapping);

  volatile uint32_t * const ddr_pair =
    reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(bank_base_u32_) + kRockchipV2PortDdrOffsetBytes);
  RockchipGpioV2WriteHalf(ddr_pair, pin_bit_, true);

  volatile uint32_t * const dr_pair =
    reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(bank_base_u32_) + kRockchipV2PortDrOffsetBytes);
  RockchipGpioV2WriteHalf(dr_pair, pin_bit_, false);

  return true;
}

void Ws2812RockchipMmap::Shutdown()
{
  bank_base_u32_ = nullptr;
  pin_bit_ = 0;
  if (mapped_ != nullptr && mapped_ != MAP_FAILED) {
    munmap(mapped_, map_length_);
    mapped_ = nullptr;
    map_length_ = 0;
  }
  if (mem_fd_ >= 0) {
    close(mem_fd_);
    mem_fd_ = -1;
  }
}

void Ws2812RockchipMmap::SetDr(bool high)
{
  if (!bank_base_u32_) {
    return;
  }
  volatile uint32_t * const dr_pair =
    reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(bank_base_u32_) + kRockchipV2PortDrOffsetBytes);
  RockchipGpioV2WriteHalf(dr_pair, pin_bit_, high);
}

void Ws2812RockchipMmap::SendReset()
{
  SetDr(false);
  std::this_thread::sleep_for(std::chrono::microseconds(timing_.reset_us));
}

void Ws2812RockchipMmap::SendByte(uint8_t byte)
{
  if (!bank_base_u32_) {
    return;
  }
  for (int i = 7; i >= 0; --i) {
    const bool bit = (byte >> i) & 1;
    if (bit) {
      SetDr(true);
      BusyDelayNs(static_cast<int64_t>(timing_.t1h_ns));
      SetDr(false);
      BusyDelayNs(static_cast<int64_t>(timing_.t1l_ns));
    } else {
      SetDr(true);
      BusyDelayNs(static_cast<int64_t>(timing_.t0h_ns));
      SetDr(false);
      BusyDelayNs(static_cast<int64_t>(timing_.t0l_ns));
    }
  }
}

bool Ws2812RockchipMmap::Show(const std::vector<uint8_t> & rgb24, unsigned int led_count)
{
  if (!bank_base_u32_) {
    return false;
  }
  const size_t expected = static_cast<size_t>(led_count) * 3U;
  if (rgb24.size() < expected) {
    return false;
  }

  SetDr(false);
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
