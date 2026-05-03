// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace trotbot_status_led
{

struct Ws2812TimingNs
{
  uint32_t t0h_ns{350};
  uint32_t t0l_ns{800};
  uint32_t t1h_ns{700};
  uint32_t t1l_ns{600};
  uint32_t reset_us{300};
  /**
   * 每帧在发首颗灯数据前，先保持低电平（微秒）。0=不启用，改由 pre_frame_idle_us。
   * WS2812 线序为 GRB，首字节是 G；首几个 bit 最容易因总线/IO 边沿被判错，常见症状为**首灯闪绿**。
   * 建议与 timing_reset_us 同量级或略大（如 400～800）。
   */
  uint32_t lead_in_reset_us{400};
  /** 当 lead_in_reset_us==0 时：每帧首字节前短低保持（微秒） */
  uint32_t pre_frame_idle_us{50};
};

}  // namespace trotbot_status_led
