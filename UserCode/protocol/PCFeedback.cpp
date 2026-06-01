/**
 * @file    PCFeedback.cpp
 * @brief   Pure chassis feedback task.
 */
#include "PCFeedback.hpp"

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace Protocol::Feedback
{
namespace
{
void write_u32(uint8_t* data, const uint32_t value)
{
    data[0] = static_cast<uint8_t>(value >> 24);
    data[1] = static_cast<uint8_t>(value >> 16);
    data[2] = static_cast<uint8_t>(value >> 8);
    data[3] = static_cast<uint8_t>(value);
}

void write_u16(uint8_t* data, const uint16_t value)
{
    data[0] = static_cast<uint8_t>(value >> 8);
    data[1] = static_cast<uint8_t>(value);
}

int16_t to_scaled_i16(const float value, const float scale)
{
    const long scaled = lroundf(value * scale);
    return static_cast<int16_t>(std::clamp(scaled,
                                           static_cast<long>(std::numeric_limits<int16_t>::min()),
                                           static_cast<long>(std::numeric_limits<int16_t>::max())));
}

uint16_t control_state()
{
    uint16_t state = static_cast<uint16_t>(Chassis::control_mode);

    if (Chassis::motion != nullptr && Chassis::motion->enabled())
        state |= 1U << 8U;
    if (Chassis::motion != nullptr && Chassis::motion->isReady())
        state |= 1U << 9U;
    if (Chassis::master_ctrl != nullptr && Chassis::master_ctrl->enabled())
        state |= 1U << 10U;
    if (Chassis::slave_ctrl != nullptr && Chassis::slave_ctrl->enabled())
        state |= 1U << 11U;

    return state;
}

uint16_t connection_state()
{
    uint16_t state = 0;

    if (Protocol::pc_rx != nullptr && Protocol::pc_rx->isConnected())
        state |= 1U << 0U;
    if (Protocol::isPcLocalizationConnected())
        state |= 1U << 1U;
    if (Protocol::isUpperHostIdentified())
        state |= 1U << 2U;

    return state;
}

constexpr uint32_t FeedbackPeriodMs = 10U;
constexpr std::array<uint8_t, HeaderLen> FeedbackHeader = { 0xAA, 0xBB };

std::array<PCProtocol*, MaxPCProtocolCount> protocols_{};
std::array<uint8_t, FeedbackFrameLen>       feedback_frame_buffer_{};

uint32_t     protocol_count_{ 0 };
osThreadId_t transmit_task_{};

void buildFeedbackFrame(std::array<uint8_t, FeedbackFrameLen>& frame)
{
    chassis::Posture posture{};
    chassis::Velocity velocity{};

    if (Chassis::loc != nullptr)
    {
        posture = Chassis::loc->postureInWorld();
        velocity = Chassis::loc->velocityInBody();
    }

    auto* payload = frame.data() + HeaderLen;

    frame[0] = FeedbackHeader[0];
    frame[1] = FeedbackHeader[1];

    write_u32(&payload[0], HAL_GetTick());
    write_u16(&payload[4], static_cast<uint16_t>(to_scaled_i16(posture.x, 2000.0f)));
    write_u16(&payload[6], static_cast<uint16_t>(to_scaled_i16(posture.y, 2000.0f)));
    write_u16(&payload[8], static_cast<uint16_t>(to_scaled_i16(posture.yaw, 100.0f)));
    write_u16(&payload[10], static_cast<uint16_t>(to_scaled_i16(velocity.vx, 2000.0f)));
    write_u16(&payload[12], static_cast<uint16_t>(to_scaled_i16(velocity.vy, 2000.0f)));
    write_u16(&payload[14], static_cast<uint16_t>(to_scaled_i16(velocity.wz, 100.0f)));
    write_u16(&payload[16], control_state());
    write_u16(&payload[18], connection_state());

    const uint16_t crc = CRC16Modbus::calc(payload, FeedbackPayloadLen - 2);
    write_u16(&payload[20], crc);
}

[[noreturn]] void TransmitTask(void* argument)
{
    (void)argument;

    for (;;)
    {
        buildFeedbackFrame(feedback_frame_buffer_);

        for (uint32_t i = 0; i < protocol_count_; ++i)
        {
            auto* protocol = protocols_[i];
            if (protocol != nullptr)
                protocol->transmitTaskStep(feedback_frame_buffer_);
        }

        osDelay(FeedbackPeriodMs);
    }
}
} // namespace

void registerProtocol(PCProtocol* protocol)
{
    assert(protocol_count_ < protocols_.size());
    protocols_[protocol_count_++] = protocol;
}

void startTask()
{
    if (transmit_task_ != nullptr)
        return;

    constexpr osThreadAttr_t feedback_attr{
        .name       = "pc-feedback",
        .stack_size = 512 * 4,
        .priority   = osPriorityLow,
    };

    transmit_task_ = osThreadNew(TransmitTask, nullptr, &feedback_attr);
}

} // namespace Protocol::Feedback
