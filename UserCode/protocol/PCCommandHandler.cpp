/**
 * @file    PCCommandHandler.cpp
 * @brief   Pure chassis command processor.
 */
#include "PCCommandHandler.hpp"

#include "RingBuffer.hpp"
#include "chassis/Config.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "project_parts.hpp"
#include "system.hpp"
#include "watchdog.hpp"

#include <cmath>

namespace Protocol
{
namespace
{
uint32_t read_u32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) << 24 | static_cast<uint32_t>(data[1]) << 16 |
           static_cast<uint32_t>(data[2]) << 8 | static_cast<uint32_t>(data[3]);
}

int16_t read_i16(const uint8_t* data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) << 8 |
                                static_cast<uint16_t>(data[1]));
}

constexpr uint32_t MsgReceived = 1U << 0U;
constexpr uint32_t LidarPostureTimeoutTicks = 200U;

libs::RingBuffer<Frame, 10> command_buffer_{};
osThreadId_t                command_handler_task_{};
Sync::Clock                 global_clock_{};
service::Watchdog           lidar_posture_watchdog_{};

float to_pos(const int16_t value)
{
    return static_cast<float>(value) / 2000.0f;
}

float to_angle(const int16_t value)
{
    return static_cast<float>(value) / 100.0f;
}

chassis::Posture read_posture(const std::array<uint8_t, 12>& data, const uint32_t offset)
{
    return { .x   = to_pos(read_i16(&data[offset])),
             .y   = to_pos(read_i16(&data[offset + 2])),
             .yaw = to_angle(read_i16(&data[offset + 4])) };
}

chassis::Velocity read_velocity(const std::array<uint8_t, 12>& data, const uint32_t offset)
{
    return { .vx = to_pos(read_i16(&data[offset])),
             .vy = to_pos(read_i16(&data[offset + 2])),
             .wz = to_angle(read_i16(&data[offset + 4])) };
}

uint16_t read_u12_hi(const uint8_t high, const uint8_t low)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(high) << 4U) |
                                 (static_cast<uint16_t>(low) >> 4U));
}

uint16_t read_u12_lo(const uint8_t high, const uint8_t low)
{
    return static_cast<uint16_t>(((static_cast<uint16_t>(high) & 0x0FU) << 8U) |
                                 static_cast<uint16_t>(low));
}

Chassis::MasterController::TrajectoryLimit read_master_limit(
        const std::array<uint8_t, 12>& data)
{
    const uint16_t xy_vmax_raw  = read_u12_hi(data[6], data[7]);
    const uint16_t xy_amax_raw  = read_u12_lo(data[7], data[8]);
    const uint16_t yaw_vmax_raw = read_u12_hi(data[9], data[10]);
    const uint16_t yaw_amax_raw = read_u12_lo(data[10], data[11]);

    const float xy_vmax  = static_cast<float>(xy_vmax_raw) / 200.0f;
    const float xy_amax  = static_cast<float>(xy_amax_raw) / 200.0f;
    const float yaw_vmax = static_cast<float>(yaw_vmax_raw);
    const float yaw_amax = static_cast<float>(yaw_amax_raw);

    auto limit = Chassis::Config::Control::DefaultTrajectoryLimit;
    if (xy_vmax > 0.0f)
    {
        limit.x.max_spd = xy_vmax;
        limit.y.max_spd = xy_vmax;
    }
    if (xy_amax > 0.0f)
    {
        limit.x.max_acc  = xy_amax;
        limit.y.max_acc  = xy_amax;
        limit.x.max_jerk = xy_amax * 50.0f;
        limit.y.max_jerk = xy_amax * 50.0f;
    }
    if (yaw_vmax > 0.0f)
        limit.yaw.max_spd = yaw_vmax;
    if (yaw_amax > 0.0f)
    {
        limit.yaw.max_acc  = yaw_amax;
        limit.yaw.max_jerk = yaw_amax * 50.0f;
    }
    return limit;
}

void handleCommand(const Frame& frame)
{
    if (frame.protocol == nullptr)
        return;

    if (frame.from_main_protocol)
    {
        const auto  self_time      = static_cast<float>(frame.rx_timestamp);
        const float target_pc_time = static_cast<float>(frame.tx_timestamp) +
                                     frame.protocol->transitionDelayMS();
        global_clock_.align(self_time, target_pc_time);
    }

    if constexpr (ProjectParts::NeedUpperHostIdentifyInit)
    {
        if (frame.from_main_protocol)
            System::Init::upperHostIdentified = true;
    }

    switch (frame.cmd)
    {
    case PCCommand::Ping:
        break;

    case PCCommand::StopChassis:
        if constexpr (ProjectParts::EnablePcControl)
            Chassis::stop();
        break;

    case PCCommand::SlavePushChassisTrajectory:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableSlaveControl)
        {
            const Chassis::SlaveController::TrajectoryPoint point{
                .p_ref = read_posture(frame.data, 0),
                .v_ref = read_velocity(frame.data, 6),
            };
            (void)Chassis::pushSlaveTrajectoryPoint(point);
        }
        break;

    case PCCommand::SetMasterChassisTargetCurrentState:
    case PCCommand::SetMasterChassisTargetPreviousCurve:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableWheelChassis)
        {
            if (!Chassis::switchToMaster() || Chassis::master_ctrl == nullptr)
                break;

            const auto target = read_posture(frame.data, 0);
            const auto limit  = read_master_limit(frame.data);
            const auto link_mode =
                    frame.cmd == PCCommand::SetMasterChassisTargetCurrentState
                            ? Chassis::MasterController::TrajectoryLinkMode::CurrentState
                            : Chassis::MasterController::TrajectoryLinkMode::PreviousCurve;

            (void)Chassis::master_ctrl->setTargetPostureInWorld(target, link_mode, limit);
        }
        break;

    case PCCommand::SetMasterChassisVelocity:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableWheelChassis)
        {
            if (!Chassis::switchToMaster() || Chassis::master_ctrl == nullptr)
                break;

            Chassis::master_ctrl->setVelocityInBody(read_velocity(frame.data, 0), false);
        }
        break;

    case PCCommand::LidarPosture:
        if constexpr (ProjectParts::EnablePcLocalization)
        {
            if (!frame.from_main_protocol || !global_clock_.isStable())
                break;

            lidar_posture_watchdog_.feed(LidarPostureTimeoutTicks);

            const auto pos             = read_posture(frame.data, 0);
            const uint32_t lidar_time  = read_u32(frame.data.data() + 6);
            const uint32_t self_time   = global_clock_.pcTime2SelfTime(lidar_time);

            if (Chassis::motion == nullptr || !Chassis::motion->isReady())
                break;

            if (!System::Init::postureReceived)
            {
                if (Device::Sensor::gyro_yaw == nullptr ||
                    !Device::Sensor::gyro_yaw->isConnected())
                    break;

                System::Init::posture = pos;
                System::Init::initPostureReceive();
                System::Init::postureReceived = true;
                break;
            }

            Chassis::updateLidar(pos, self_time);
        }
        break;
    }
}

[[noreturn]] void PCCommandHandlerTask(void* argument)
{
    (void)argument;

    for (;;)
    {
        Frame frame{};
        while (command_buffer_.pop(frame))
            handleCommand(frame);

        osThreadFlagsWait(MsgReceived, osFlagsWaitAny, osWaitForever);
    }
}
} // namespace

const Sync::Clock& clock()
{
    return global_clock_;
}

bool isPcLocalizationConnected()
{
    return lidar_posture_watchdog_.isFed();
}

bool isUpperHostIdentified()
{
    return !ProjectParts::NeedUpperHostIdentifyInit || System::Init::upperHostIdentified;
}

namespace CommandHandler
{

bool enqueueFrame(const Frame& frame)
{
    const bool pushed = command_buffer_.push(frame);
    if (pushed && command_handler_task_ != nullptr)
        osThreadFlagsSet(command_handler_task_, MsgReceived);

    return pushed;
}

void startTask()
{
    if (command_handler_task_ != nullptr)
        return;

    constexpr osThreadAttr_t processor_attr{
        .name       = "pc-cmd-processor",
        .stack_size = 4096 * 4,
        .priority   = osPriorityRealtime,
    };

    command_handler_task_ = osThreadNew(PCCommandHandlerTask, nullptr, &processor_attr);
}

} // namespace CommandHandler
} // namespace Protocol
