/**
 * @file    Config.hpp
 * @brief   Pure mecanum chassis parameters.
 */
#pragma once

#include "Slave.hpp"
#include "Mecanum4.hpp"
#include "pid_motor.hpp"

namespace Chassis::Config
{

constexpr float sq(const float value)
{
    return value * value;
}

namespace Motion
{
constexpr PIDMotor::Config MotorWheelVelPIDCfg{
    .Kp             = 0.0f,
    .Ki             = 0.0f,
    .Kd             = 0.0f,
    .abs_output_max = 0.0f,
};

constexpr float WheelRadiusMM    = 50.0f;
constexpr float WheelDistanceXMM = 458.1f;
constexpr float WheelDistanceYMM = 401.08f;
constexpr auto  ChassisType      = chassis::motion::Mecanum4::ChassisType::XType;
} // namespace Motion

namespace Control
{
using SlavePDConfig = chassis::controller::Slave<64>::PDConfig;

constexpr SlavePDConfig slavePDCfg{
    .vx = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 0.6f },
    .vy = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 0.6f },
    .wz = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 135.0f },
};
} // namespace Control

namespace Loc
{
constexpr float InitCovXY        = sq(0.1f);
constexpr float InitCovYaw       = sq(0.1f);
constexpr float InitCovYawOffset = sq(10.0f);

constexpr float ProcessNoiseXY        = sq(0.05f);
constexpr float ProcessNoiseYaw       = sq(0.5f);
constexpr float ProcessNoiseYawOffset = sq(0.01f);

constexpr float GyroYawNoise  = sq(0.1f);
constexpr float LidarXYNoise  = sq(0.01f);
constexpr float LidarYawNoise = sq(0.5f);
} // namespace Loc

} // namespace Chassis::Config
