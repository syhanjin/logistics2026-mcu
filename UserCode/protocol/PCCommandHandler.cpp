/**
 * @file    PCCommandHandler.cpp
 * @brief   Pure chassis command processor.
 */
#include "PCCommandHandler.hpp"

#include "RingBuffer.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "project_parts.hpp"
#include "system.hpp"
#include "watchdog.hpp"

namespace Protocol
{
namespace
{
constexpr uint32_t MsgReceived = 1U << 0U;
constexpr uint32_t LidarPostureTimeoutTicks = 200U;

libs::RingBuffer<LidarPostureFrame, 10>     lidar_posture_buffer_{};
libs::RingBuffer<UpperHostControlFrame, 10> control_buffer_{};
osThreadId_t                               command_handler_task_{};
Sync::Clock                                global_clock_{};
service::Watchdog                          lidar_posture_watchdog_{};

void notifyTask()
{
    if (command_handler_task_ != nullptr)
        osThreadFlagsSet(command_handler_task_, MsgReceived);
}

void handleLidarPostureFrame(const LidarPostureFrame& frame)
{
    if constexpr (ProjectParts::EnablePcLocalization)
    {
        if (frame.protocol == nullptr)
            return;

        const auto  self_time      = static_cast<float>(frame.rx_timestamp);
        const float target_pc_time = static_cast<float>(frame.tx_timestamp) +
                                     frame.protocol->transitionDelayMS();
        global_clock_.align(self_time, target_pc_time);

        if (!global_clock_.isStable())
            return;

        lidar_posture_watchdog_.feed(LidarPostureTimeoutTicks);

        if (Chassis::motion == nullptr || !Chassis::motion->isReady())
            return;

        if (!System::Init::postureReceived)
        {
            if (Device::Sensor::gyro_yaw == nullptr || !Device::Sensor::gyro_yaw->isConnected())
                return;

            System::Init::posture = frame.posture;
            System::Init::initPostureReceive();
            System::Init::postureReceived = true;
            return;
        }

        const uint32_t self_lidar_time = global_clock_.pcTime2SelfTime(frame.lidar_timestamp);
        Chassis::updateLidar(frame.posture, self_lidar_time);
    }
}

void handleUpperHostControlFrame(const UpperHostControlFrame& frame)
{
    if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableSlaveControl)
    {
        const Chassis::SlaveController::TrajectoryPoint point{
                .p_ref = { .x = frame.x, .y = frame.y, .yaw = frame.yaw },
                .v_ref = { .vx = frame.dx, .vy = frame.dy, .wz = frame.dyaw },
        };
        (void)Chassis::pushSlaveTrajectoryPoint(point);
    }
}

[[noreturn]] void PCCommandHandlerTask(void* argument)
{
    (void)argument;

    for (;;)
    {
        LidarPostureFrame lidar_frame{};
        while (lidar_posture_buffer_.pop(lidar_frame))
            handleLidarPostureFrame(lidar_frame);

        UpperHostControlFrame control_frame{};
        while (control_buffer_.pop(control_frame))
            handleUpperHostControlFrame(control_frame);

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

namespace CommandHandler
{

bool enqueueLidarPostureFrame(const LidarPostureFrame& frame)
{
    const bool pushed = lidar_posture_buffer_.push(frame);
    if (pushed)
        notifyTask();

    return pushed;
}

bool enqueueUpperHostControlFrame(const UpperHostControlFrame& frame)
{
    const bool pushed = control_buffer_.push(frame);
    if (pushed)
        notifyTask();

    return pushed;
}

void startTask()
{
    if (command_handler_task_ != nullptr)
        return;

    constexpr osThreadAttr_t processor_attr{
            .name       = "pc-command",
            .stack_size = 1024 * 4,
            .priority   = osPriorityRealtime,
    };

    command_handler_task_ = osThreadNew(PCCommandHandlerTask, nullptr, &processor_attr);
}

} // namespace CommandHandler
} // namespace Protocol
