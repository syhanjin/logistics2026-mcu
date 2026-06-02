/**
 * @file    PCProtocol.cpp
 * @brief   Pure chassis upper-host protocol receiver.
 */
#include "PCProtocol.hpp"

#include "PCCommandHandler.hpp"
#include "device.hpp"
#include "project_parts.hpp"

#include <cassert>
#include <cstring>

namespace Protocol
{
namespace
{
uint16_t read_u16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(data[1]) << 8;
}

uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | static_cast<uint32_t>(data[1]) << 8 |
           static_cast<uint32_t>(data[2]) << 16 | static_cast<uint32_t>(data[3]) << 24;
}

float read_f32_le(const uint8_t* data)
{
    const uint32_t raw = read_u32_le(data);
    float          value{};
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}
} // namespace

LidarPostureProtocol::LidarPostureProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

bool LidarPostureProtocol::decode(const uint8_t data[LidarPayloadLen])
{
    const uint16_t crc_in_data = read_u16_le(&data[LidarPayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, LidarPayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    LidarPostureFrame frame{};
    frame.protocol        = this;
    frame.rx_timestamp    = HAL_GetTick();
    frame.posture.x       = read_f32_le(&data[0]);
    frame.posture.y       = read_f32_le(&data[4]);
    frame.posture.yaw     = read_f32_le(&data[8]);
    frame.lidar_timestamp = read_u32_le(&data[12]);
    frame.tx_timestamp    = read_u32_le(&data[16]);
    frame.crc16           = crc_in_data;

    return CommandHandler::enqueueLidarPostureFrame(frame);
}

UpperHostControlProtocol::UpperHostControlProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

bool UpperHostControlProtocol::decode(const uint8_t data[ControlPayloadLen])
{
    const uint16_t crc_in_data = read_u16_le(&data[ControlPayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, ControlPayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    UpperHostControlFrame frame{};
    frame.rx_timestamp = HAL_GetTick();

    uint32_t offset = 0;
    frame.x         = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.y = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.yaw = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dx = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dy = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dyaw = read_f32_le(&data[offset]);
    offset += sizeof(float);

    frame.h = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dh = read_f32_le(&data[offset]);
    offset += sizeof(float);

    frame.q1 = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.q2 = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dq1 = read_f32_le(&data[offset]);
    offset += sizeof(float);
    frame.dq2 = read_f32_le(&data[offset]);
    offset += sizeof(float);

    frame.angle1 = read_u16_le(&data[offset]);
    offset += sizeof(uint16_t);
    frame.angle2 = read_u16_le(&data[offset]);
    offset += sizeof(uint16_t);
    frame.flags = data[offset];
    frame.crc16 = crc_in_data;

    return CommandHandler::enqueueUpperHostControlFrame(frame);
}

void init()
{
    if constexpr (!ProjectParts::EnableUpperHostProtocol)
        return;

    if (lidar_posture_rx != nullptr || upper_host_control_rx != nullptr)
        return;

    assert(config::uart::LidarPostureHost->Init.BaudRate == 230400);
    assert(config::uart::UpperHostControl->Init.BaudRate == 230400);

    if constexpr (ProjectParts::EnablePcLocalization)
    {
        lidar_posture_rx = new LidarPostureProtocol(config::uart::LidarPostureHost);

        HAL_UART_RegisterCallback(config::uart::LidarPostureHost,
                                  HAL_UART_RX_COMPLETE_CB_ID,
                                  [](UART_HandleTypeDef*) { lidar_posture_rx->receiveCallback(); });
        HAL_UART_RegisterCallback(config::uart::LidarPostureHost,
                                  HAL_UART_ERROR_CB_ID,
                                  [](UART_HandleTypeDef*) { lidar_posture_rx->errorHandler(); });
    }

    if constexpr (ProjectParts::EnablePcControl || ProjectParts::EnableSlaveControl)
    {
        upper_host_control_rx = new UpperHostControlProtocol(config::uart::UpperHostControl);

        HAL_UART_RegisterCallback(config::uart::UpperHostControl,
                                  HAL_UART_RX_COMPLETE_CB_ID,
                                  [](UART_HandleTypeDef*) {
                                      upper_host_control_rx->receiveCallback();
                                  });
        HAL_UART_RegisterCallback(config::uart::UpperHostControl,
                                  HAL_UART_ERROR_CB_ID,
                                  [](UART_HandleTypeDef*) { upper_host_control_rx->errorHandler(); });
    }

    CommandHandler::startTask();

    bool started = true;
    if constexpr (ProjectParts::EnablePcLocalization)
        started = started && lidar_posture_rx->startReceive();
    if constexpr (ProjectParts::EnablePcControl || ProjectParts::EnableSlaveControl)
        started = started && upper_host_control_rx->startReceive();

    if (!started)
        Error_Handler();
}

} // namespace Protocol
