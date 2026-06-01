/**
 * @file    Clock.hpp
 * @brief   Upper-host to local clock alignment.
 */
#pragma once

#include "stm32f4xx_hal.h"

#include <cmath>
#include <cstdint>

namespace Sync
{

class Clock
{
public:
    [[nodiscard]] uint32_t pcTime2SelfTime(const uint32_t pc_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(pc_time) + 0.5f - offset_);
    }

    [[nodiscard]] uint32_t selfTime2PCTime(const uint32_t self_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(self_time) + 0.5f + offset_);
    }

    [[nodiscard]] uint32_t getPCTime() const { return selfTime2PCTime(HAL_GetTick()); }
    [[nodiscard]] bool     isStable() const { return stable_; }

    void align(const float self_time, const float pc_time)
    {
        const float delta = pc_time - self_time;
        offset_           = offset_ * (1.0f - alpha) + delta * alpha;
        stable_           = std::fabs(pc_time - selfTime2PCTime(static_cast<uint32_t>(self_time))) <=
                  stable_threshold_ms_;
    }

private:
    float offset_{ 0.0f };
    bool  stable_{ false };

    static constexpr float alpha               = 0.2f;
    static constexpr float stable_threshold_ms_ = 20.0f;
};

} // namespace Sync
