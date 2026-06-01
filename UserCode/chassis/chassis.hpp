/**
 * @file    chassis.hpp
 * @brief   Pure mecanum chassis orchestration.
 */
#pragma once

#include "IChassisLoc.hpp"
#include "JustEncoder.hpp"
#include "LocEKF.hpp"
#include "Master.hpp"
#include "Mecanum4.hpp"
#include "Slave.hpp"
#include "motor_vel_controller.hpp"

#include <cstdint>

namespace Chassis
{

using ChassisMotion     = chassis::motion::Mecanum4;
using ChassisLoc        = chassis::loc::IChassisLoc;
using ChassisLocEKF     = chassis::loc::LocEKF<256>;
using ChassisLocEncoder = chassis::loc::JustEncoder;
using MasterController  = chassis::controller::Master;
using SlaveController   = chassis::controller::Slave<64>;

enum class ControlMode : uint8_t
{
    None,
    Master,
    Slave,
};

inline ChassisMotion*     motion{};
inline ChassisLoc*        loc{};
inline ChassisLocEKF*     loc_ekf{};
inline ChassisLocEncoder* loc_encoder{};
inline MasterController*  master_ctrl{};
inline SlaveController*   slave_ctrl{};
inline ControlMode        control_mode{ ControlMode::None };

void init();

void initLocCtrl(const chassis::Posture& init_posture);
void initStandaloneLocCtrl();

void updateLidar(const chassis::Posture& posture, uint32_t ticks);

void enable();
void stop();
bool switchToMaster();
bool switchToSlave();
bool pushSlaveTrajectoryPoint(const SlaveController::TrajectoryPoint& point);

void update_1kHz();
void update_100Hz();

} // namespace Chassis
