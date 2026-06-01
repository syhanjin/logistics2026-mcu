/**
 * @file    chassis.cpp
 * @brief   Pure mecanum chassis orchestration.
 */
#include "chassis.hpp"

#include "Config.hpp"
#include "device.hpp"
#include "project_parts.hpp"
#include "system.hpp"

namespace Chassis
{
namespace
{
controllers::MotorVelController* wheel_ctrl[4]{};

ChassisLocEKF::Config make_loc_ekf_config(const chassis::Posture& init_posture)
{
    const float init_gyro_yaw = Device::Sensor::gyro_yaw->getYaw();

    return {
        .x_init = { .x          = init_posture.x,
                    .y          = init_posture.y,
                    .yaw        = init_gyro_yaw,
                    .yaw_offset = init_posture.yaw - init_gyro_yaw },
        .covP   = { .xy         = Config::Loc::InitCovXY,
                    .yaw        = Config::Loc::InitCovYaw,
                    .yaw_offset = Config::Loc::InitCovYawOffset },
        .noiseQ = { .xy         = Config::Loc::ProcessNoiseXY,
                    .yaw        = Config::Loc::ProcessNoiseYaw,
                    .yaw_offset = Config::Loc::ProcessNoiseYawOffset },
        .noiseR = {
                .gyro  = { .yaw = Config::Loc::GyroYawNoise },
                .lidar = { .xy = Config::Loc::LidarXYNoise,
                           .yaw = Config::Loc::LidarYawNoise },
        },
    };
}

[[nodiscard]] constexpr chassis::Posture default_init_posture()
{
    return { .x = 0.0f, .y = 0.0f, .yaw = 0.0f };
}

controllers::MotorVelController::Config wheel_ctrl_config()
{
    return {
        .pid                = Config::Motion::MotorWheelVelPIDCfg,
        .ctrl_mode          = controllers::ControlMode::InternalVel,
        .internal_set_ratio = 10,
    };
}
} // namespace

void init()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    for (auto* motor : Device::Motor::wheel)
    {
        if (motor == nullptr)
            Error_Handler();
    }

    const auto cfg = wheel_ctrl_config();
    for (size_t i = 0; i < 4; ++i)
        wheel_ctrl[i] = new controllers::MotorVelController(Device::Motor::wheel[i], cfg);

    motion = new ChassisMotion({
            .wheel_radius      = Config::Motion::WheelRadiusMM,
            .wheel_distance_x  = Config::Motion::WheelDistanceXMM,
            .wheel_distance_y  = Config::Motion::WheelDistanceYMM,
            .chassis_type      = Config::Motion::ChassisType,
            .wheel_front_right = wheel_ctrl[0],
            .wheel_front_left  = wheel_ctrl[1],
            .wheel_rear_left   = wheel_ctrl[2],
            .wheel_rear_right  = wheel_ctrl[3],
    });
}

void initLocCtrl(const chassis::Posture& init_posture)
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    if (loc != nullptr || master_ctrl != nullptr || slave_ctrl != nullptr || motion == nullptr)
        return;

    if constexpr (ProjectParts::EnableEkfLocalization)
    {
        if (Device::Sensor::gyro_yaw == nullptr)
            return;

        loc_ekf = new ChassisLocEKF(*motion,
                                    make_loc_ekf_config(init_posture),
                                    *Device::Sensor::gyro_yaw,
                                    1);
        loc     = loc_ekf;
    }
    else if constexpr (ProjectParts::EnableJustEncoderLocalization)
    {
        loc_encoder = new chassis::loc::JustEncoder(*motion);
        loc         = loc_encoder;
    }

    if (loc == nullptr)
        return;

    master_ctrl = new MasterController(*motion, *loc, Config::Control::masterCfg);
    if constexpr (ProjectParts::EnableSlaveControl)
        slave_ctrl = new SlaveController(*motion, *loc, Config::Control::slavePDCfg);
}

void initStandaloneLocCtrl()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    if constexpr (ProjectParts::NeedUpperHostInitPosture)
        return;

    initLocCtrl(default_init_posture());
}

void updateLidar(const chassis::Posture& posture, const uint32_t ticks)
{
    if (loc_ekf != nullptr)
        loc_ekf->updateLidar(posture, ticks);
}

void enable()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    if (master_ctrl == nullptr || !master_ctrl->enable())
        Error_Handler();

    control_mode = ControlMode::Master;
}

void stop()
{
    if (master_ctrl != nullptr)
        master_ctrl->stop();
    if (slave_ctrl != nullptr)
    {
        slave_ctrl->stop();
        slave_ctrl->clearTrajectory();
    }
}

bool switchToMaster()
{
    if (master_ctrl == nullptr)
        return false;

    if (slave_ctrl != nullptr)
    {
        slave_ctrl->stop();
        slave_ctrl->clearTrajectory();
        slave_ctrl->releaseControl();
    }

    const bool acquired = master_ctrl->acquireControl();
    if (acquired)
        control_mode = ControlMode::Master;
    return acquired;
}

bool switchToSlave()
{
    if (slave_ctrl == nullptr)
        return false;

    if (master_ctrl != nullptr)
    {
        master_ctrl->stop();
        master_ctrl->releaseControl();
    }

    slave_ctrl->clearTrajectory();
    const bool acquired = slave_ctrl->acquireControl();
    if (acquired)
        control_mode = ControlMode::Slave;
    return acquired;
}

bool pushSlaveTrajectoryPoint(const SlaveController::TrajectoryPoint& point)
{
    if (!switchToSlave())
        return false;

    return slave_ctrl->pushTrajectoryPoint(point);
}

void update_1kHz()
{
    static uint32_t master_error_prescaler_500Hz = 0;

    if (loc_ekf != nullptr)
        loc_ekf->update();
    else if (loc_encoder != nullptr)
        loc_encoder->update(0.001f);

    if (control_mode == ControlMode::Master && master_ctrl != nullptr)
    {
        ++master_error_prescaler_500Hz;
        if (master_error_prescaler_500Hz >= 2)
        {
            master_ctrl->errorUpdate();
            master_error_prescaler_500Hz = 0;
        }
        master_ctrl->controllerUpdate();
    }
    else if (control_mode == ControlMode::Slave && slave_ctrl != nullptr)
    {
        slave_ctrl->trajectoryUpdate();
        slave_ctrl->errorUpdate();
    }

    if (motion != nullptr)
        motion->update();
}

void update_100Hz()
{
    if (control_mode == ControlMode::Master && master_ctrl != nullptr)
        master_ctrl->profileUpdate(0.01f);
}

} // namespace Chassis

void System::Init::initPostureReceive()
{
    Chassis::initLocCtrl(posture);
}
