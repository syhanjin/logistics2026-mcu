/**
 * @file    PCCommandDef.hpp
 * @brief   Pure chassis upper-host protocol definitions.
 */
#pragma once

#include "crc.hpp"

#include <cstdint>

using CRC16Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen  = 2;
constexpr uint32_t PayloadLen = 1 + 2 * 6 + 4 + 2;
constexpr uint32_t FrameLen   = HeaderLen + PayloadLen;

constexpr uint8_t IdentifyInitByte = 0xAA;

/// AA BB | timestamp(uint32) | x*2000(int16) | y*2000(int16) | yaw*100(int16) |
/// vx*2000(int16) | vy*2000(int16) | wz*100(int16) | control(uint16) |
/// connection(uint16) | CRC16
constexpr uint32_t FeedbackPayloadLen = 4 + 2 * 8 + 2;
constexpr uint32_t FeedbackFrameLen   = HeaderLen + FeedbackPayloadLen;

enum class PCCommand : uint8_t
{
    Ping = 0x01,

    StopChassis = 0x10,

    SlavePushChassisTrajectory       = 0x12,
    SetMasterChassisTargetCurrentState = 0x13,
    SetMasterChassisTargetPreviousCurve = 0x14,
    SetMasterChassisVelocity           = 0x15,

    LidarPosture = 0x21,
};
