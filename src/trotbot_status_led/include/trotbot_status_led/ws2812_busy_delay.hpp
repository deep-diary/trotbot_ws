// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>

namespace trotbot_status_led
{

#if defined(__aarch64__)

inline uint64_t ReadCntfrqEl0()
{
  uint64_t v = 0;
  asm volatile("mrs %0, cntfrq_el0" : "=r"(v));
  return v;
}

inline uint64_t ReadCntvctEl0()
{
  uint64_t v = 0;
  asm volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(v) :: "memory");
  return v;
}

/** 用 CNTVCT 忙等；比 steady_clock 更适合亚微秒 WS2812 时序（避免全程被判成 1 → 发白） */
inline void BusyDelayNs(int64_t ns)
{
  if (ns <= 0) {
    return;
  }
  static uint64_t freq_hz = 0;
  if (freq_hz == 0) {
    freq_hz = ReadCntfrqEl0();
    if (freq_hz < 1000000ULL) {
      freq_hz = 24000000ULL;
    }
  }
  const uint64_t ticks =
    static_cast<uint64_t>(static_cast<uint64_t>(ns) * freq_hz / 1000000000ULL);
  const uint64_t start = ReadCntvctEl0();
  while (ReadCntvctEl0() - start < ticks) {
    asm volatile("" ::: "memory");
  }
}

#else

inline void BusyDelayNs(int64_t ns)
{
  if (ns <= 0) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const int64_t elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count();
    if (elapsed >= ns) {
      break;
    }
  }
}

#endif

}  // namespace trotbot_status_led
