/**
 * @file    PCCommandDef.hpp
 * @brief   Pure chassis upper-host protocol definitions.
 */
#pragma once

#include "crc.hpp"

#include <cstdint>

using CRC16Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen = 2;

constexpr uint32_t LidarPayloadLen = 3 * sizeof(float) + 2 * sizeof(uint32_t) + 2;
constexpr uint32_t LidarFrameLen   = HeaderLen + LidarPayloadLen;

constexpr uint32_t ControlPayloadLen = 6 * sizeof(float) + 2 * sizeof(float) +
                                       4 * sizeof(float) + 2 * sizeof(uint16_t) +
                                       sizeof(uint8_t) + 2;
constexpr uint32_t ControlFrameLen = HeaderLen + ControlPayloadLen;
