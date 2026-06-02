/**
 * @file    PCProtocol.hpp
 * @brief   Pure chassis upper-host protocol receiver.
 */
#pragma once

#include "IChassisDef.hpp"
#include "PCCommandDef.hpp"
#include "UartRxSync.hpp"
#include "sync/Clock.hpp"

#include <array>
#include <cstdint>

namespace Protocol
{

class LidarPostureProtocol;
class UpperHostControlProtocol;

struct LidarPostureFrame
{
    LidarPostureProtocol* protocol{};
    uint32_t              rx_timestamp{};
    chassis::Posture      posture{};
    uint32_t              lidar_timestamp{};
    uint32_t              tx_timestamp{};
    uint16_t              crc16{};
};

struct UpperHostControlFrame
{
    uint32_t rx_timestamp{};

    float x{};
    float y{};
    float yaw{};
    float dx{};
    float dy{};
    float dyaw{};

    float h{};
    float dh{};

    float q1{};
    float q2{};
    float dq1{};
    float dq2{};

    uint16_t angle1{};
    uint16_t angle2{};
    uint8_t  flags{};

    uint16_t crc16{};
};

class LidarPostureProtocol final : public protocol::UartRxSync<HeaderLen, LidarFrameLen>
{
public:
    explicit LidarPostureProtocol(UART_HandleTypeDef* huart);

    [[nodiscard]] float transitionDelayMS() const
    {
        return static_cast<float>(LidarFrameLen) * 10.0f * 1000.0f /
               static_cast<float>(huart()->Init.BaudRate);
    }

protected:
    static constexpr std::array<uint8_t, HeaderLen> HEADER = { 0xAA, 0xBB };

    [[nodiscard]] const std::array<uint8_t, HeaderLen>& header() const override { return HEADER; }

    bool decode(const uint8_t data[LidarPayloadLen]) override;

    [[nodiscard]] uint32_t timeout() const override { return 250; }
};

class UpperHostControlProtocol final : public protocol::UartRxSync<HeaderLen, ControlFrameLen>
{
public:
    explicit UpperHostControlProtocol(UART_HandleTypeDef* huart);

protected:
    static constexpr std::array<uint8_t, HeaderLen> HEADER = { 0xAA, 0xBB };

    [[nodiscard]] const std::array<uint8_t, HeaderLen>& header() const override { return HEADER; }

    bool decode(const uint8_t data[ControlPayloadLen]) override;

    [[nodiscard]] uint32_t timeout() const override { return 250; }
};

inline LidarPostureProtocol*    lidar_posture_rx{};
inline UpperHostControlProtocol* upper_host_control_rx{};

[[nodiscard]] const Sync::Clock& clock();

bool isPcLocalizationConnected();

void init();

} // namespace Protocol
