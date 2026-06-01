/**
 * @file    Config.hpp
 * @brief   Pure mecanum chassis parameters.
 */
#pragma once

#include "Master.hpp"
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
using MasterConfig                = chassis::controller::Master::Config;
using SlavePDConfig               = chassis::controller::Slave<64>::PDConfig;
using TrajectoryLimit             = chassis::controller::Master::TrajectoryLimit;
using TrajectoryTrackingThreshold = chassis::controller::Master::TrajectoryTrackingThreshold;

constexpr TrajectoryLimit MaxTrajectoryLimit{
    .x   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .y   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .yaw = { .max_spd = 460.0f, .max_acc = 170.0f, .max_jerk = 8500.0f },
};

constexpr float TrajectoryLimitRatio = 0.7f;

constexpr TrajectoryLimit DefaultTrajectoryLimit = MaxTrajectoryLimit * TrajectoryLimitRatio;

constexpr TrajectoryTrackingThreshold DefaultTrajectoryTrackingThreshold{
    .x   = 0.01f,
    .y   = 0.01f,
    .yaw = 0.5f,
};

constexpr MasterConfig masterCfg{
    .posture_error_pd_cfg = {
            .vx = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 0.6f },
            .vy = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 0.6f },
            .wz = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 135.0f },
    },
    .limit              = DefaultTrajectoryLimit,
    .tracking_threshold = DefaultTrajectoryTrackingThreshold,
};

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
