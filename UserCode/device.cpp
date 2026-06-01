/**
 * @file    device.cpp
 * @brief   Pure chassis device initialization.
 */
#include "device.hpp"

#include "can.h"
#include "can_driver.h"
#include "project_parts.hpp"

namespace Device
{
namespace
{
constexpr uint32_t DMFeedbackMasterId = 0x00;

namespace S3519
{
// These limits must match the Damiao upper-tool driver configuration.
constexpr float PosMaxRad = 12.5f;
constexpr float VelMaxRad = 45.0f;
constexpr float TorMaxNm  = 18.0f;
} // namespace S3519

void sensor_init()
{
    if constexpr (!ProjectParts::EnableGyro)
        return;

    Sensor::gyro_yaw = new sensors::gyro::HWT101CT(config::uart::SensorGyroYaw);
    UartRxSync_RegisterCallback(Sensor::gyro_yaw, config::uart::SensorGyroYaw);

    if (!Sensor::gyro_yaw->startReceive())
        Error_Handler();
}

void can_init()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    motors::DMMotor::CAN_FilterInit(&hcan1, 0, DMFeedbackMasterId);
    motors::DMMotor::CAN_FilterInit(&hcan2, 14, DMFeedbackMasterId);

    CAN_RegisterCallback(&hcan1, motors::DMMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan2, motors::DMMotor::CANBaseReceiveCallback);

    CAN_InitMainCallback(&hcan1);
    CAN_InitMainCallback(&hcan2);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DMMotor::Config wheel_motor_config[4] = {
    {
            .hcan        = &hcan1,
            .id0         = 0x09,
            .type        = motors::DMMotor::Type::S3519,
            .mode        = motors::DMMotor::Mode::Vel,
            .pos_max_rad = S3519::PosMaxRad,
            .vel_max_rad = S3519::VelMaxRad,
            .tor_max     = S3519::TorMaxNm,
    },
    {
            .hcan        = &hcan1,
            .id0         = 0x0A,
            .type        = motors::DMMotor::Type::S3519,
            .mode        = motors::DMMotor::Mode::Vel,
            .pos_max_rad = S3519::PosMaxRad,
            .vel_max_rad = S3519::VelMaxRad,
            .tor_max     = S3519::TorMaxNm,
    },
    {
            .hcan        = &hcan2,
            .id0         = 0x0B,
            .type        = motors::DMMotor::Type::S3519,
            .mode        = motors::DMMotor::Mode::Vel,
            .pos_max_rad = S3519::PosMaxRad,
            .vel_max_rad = S3519::VelMaxRad,
            .tor_max     = S3519::TorMaxNm,
    },
    {
            .hcan        = &hcan2,
            .id0         = 0x0C,
            .type        = motors::DMMotor::Type::S3519,
            .mode        = motors::DMMotor::Mode::Vel,
            .pos_max_rad = S3519::PosMaxRad,
            .vel_max_rad = S3519::VelMaxRad,
            .tor_max     = S3519::TorMaxNm,
    },
};

void wheel_motor_init()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    for (size_t i = 0; i < 4; ++i)
        Motor::wheel[i] = new motors::DMMotor(wheel_motor_config[i]);
}
} // namespace

void init()
{
    sensor_init();
    can_init();
    wheel_motor_init();
}

void update_1kHz()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    for (auto* motor : Motor::wheel)
    {
        if (motor != nullptr)
            motor->ping();
    }
}

} // namespace Device
