/**
 * @file    device.hpp
 * @brief   Pure chassis device declarations.
 */
#pragma once

#include "HWT101CT.hpp"
#include "dm.hpp"
#include "usart.h"

#include <cstddef>

namespace config::uart
{
constexpr auto UpperHostControl = &huart1;
constexpr auto SensorGyroYaw     = &huart2;
constexpr auto LidarPostureHost  = &huart3;
} // namespace config::uart

namespace Device
{

namespace Sensor
{
inline sensors::gyro::HWT101CT* gyro_yaw{};
} // namespace Sensor

namespace Motor
{
inline motors::DMMotor* wheel[4]{};
} // namespace Motor

void init();
void update_1kHz();

} // namespace Device
