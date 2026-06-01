/**
 * @file    project_parts.hpp
 * @brief   Pure chassis capability switches.
 */
#pragma once

#ifndef PROJECT_PART_ENABLE_WHEEL_CHASSIS
#    define PROJECT_PART_ENABLE_WHEEL_CHASSIS 1
#endif

#ifndef PROJECT_PART_ENABLE_GYRO
#    define PROJECT_PART_ENABLE_GYRO 1
#endif

#ifndef PROJECT_PART_ENABLE_PC_LOCALIZATION
#    define PROJECT_PART_ENABLE_PC_LOCALIZATION 1
#endif

#ifndef PROJECT_PART_ENABLE_PC_CONTROL
#    define PROJECT_PART_ENABLE_PC_CONTROL 1
#endif

#ifndef PROJECT_PART_ENABLE_SLAVE_CONTROL
#    define PROJECT_PART_ENABLE_SLAVE_CONTROL 1
#endif

#ifndef PROJECT_PART_ENABLE_UPPER_HOST_IDENTIFY_INIT
#    define PROJECT_PART_ENABLE_UPPER_HOST_IDENTIFY_INIT 0
#endif

namespace ProjectParts
{

inline constexpr bool EnableWheelChassis = PROJECT_PART_ENABLE_WHEEL_CHASSIS != 0;
inline constexpr bool EnableGyro         = PROJECT_PART_ENABLE_GYRO != 0;
inline constexpr bool EnablePcLocalization = PROJECT_PART_ENABLE_PC_LOCALIZATION != 0;
inline constexpr bool EnablePcControl      = PROJECT_PART_ENABLE_PC_CONTROL != 0;
inline constexpr bool EnableSlaveControl   = PROJECT_PART_ENABLE_SLAVE_CONTROL != 0;
inline constexpr bool EnableUpperHostIdentifyInit =
        PROJECT_PART_ENABLE_UPPER_HOST_IDENTIFY_INIT != 0;

inline constexpr bool EnableUpperHostProtocol =
        EnablePcLocalization || EnablePcControl || EnableSlaveControl;
inline constexpr bool NeedUpperHostIdentifyInit =
        EnableUpperHostProtocol && EnableUpperHostIdentifyInit;

inline constexpr bool EnableChassisLocalization = EnableWheelChassis;
inline constexpr bool EnableJustEncoderLocalization = EnableChassisLocalization && !EnableGyro;
inline constexpr bool EnableEkfLocalization         = EnableChassisLocalization && EnableGyro;
inline constexpr bool NeedUpperHostInitPosture      = EnablePcLocalization;

static_assert(!EnablePcLocalization || EnableEkfLocalization,
              "PROJECT_PART_ENABLE_PC_LOCALIZATION requires wheel chassis and gyro enabled.");
static_assert(EnablePcControl || !EnableSlaveControl,
              "PROJECT_PART_ENABLE_SLAVE_CONTROL requires PROJECT_PART_ENABLE_PC_CONTROL enabled.");

} // namespace ProjectParts
