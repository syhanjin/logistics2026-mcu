# Pure Mecanum Chassis Port

## Goal

Extend the current `chassis` CubeMX/CMake skeleton into a pure chassis lower-controller firmware, using the chassis implementation in `/home/syhanjin/workspace/robocon2026/robot_gen2/26-v2-r2-ind-mcu-chassis/` as the reference while removing non-chassis capabilities such as grip, lift, suction, infrared, connection table, and step actions.

## Required Capabilities

- Four-wheel mecanum chassis motion control using the library-provided `Chassis::Mecanum4`.
- EKF localization that fuses wheel/chassis kinematics, gyro yaw, and upper-host `LidarPosture` external observations.
- Master and Slave chassis control modes.
- Pure chassis serial protocol covering upper-host localization, velocity/target control, Slave trajectory points, and chassis feedback.
- Wheel motors use `motors::DMMotor` with `DM::S3519`.
- Wheel order and CAN distribution:
  - `wheel[0]` front-right: CAN1, `id0 = 0x09`
  - `wheel[1]` front-left: CAN1, `id0 = 0x0A`
  - `wheel[2]` rear-left: CAN2, `id0 = 0x0B`
  - `wheel[3]` rear-right: CAN2, `id0 = 0x0C`

## Scope

### Modules

- Bring in the needed `Modules/ChassisController` pieces:
  - `Core`
  - `Chassis/Mecanum4`
  - `Localization/EKF`
  - `Localization/JustEncoder`
  - `Controller/Master`
  - `Controller/Slave`
- Bring in the needed `Modules/MotorDrivers` pieces:
  - `core`
  - `controllers`
  - `motors/DM`
  - Do not introduce DJI/VESC/MI usage unless the existing CMake/module layout requires minimal directory-level glue.
- Bring in `Modules/Sensors/gyro/HWT101CT` for EKF yaw input.
- Bring in `VelocityProfile/SCurve` and any math/control foundation required by the Master controller.

### CubeMX / Generated Peripheral Configuration

- `CAN1` and `CAN2`: 1 Mbps, RX/TX interrupts enabled, HAL callback registration enabled.
- `USART1`: auxiliary controller / Slave protocol entry, 230400 baud, RX circular DMA, TX normal DMA.
- `USART2`: HWT101CT yaw gyro.
- `USART3`: main upper-host protocol entry, 230400 baud, RX circular DMA, TX normal DMA.
- `DMA`: configured for UART RX/TX DMA and `UartRxSync` requirements.
- `TIM5`: 1 kHz control tick. Use period elapsed plus output compare half-cycle callbacks to split control calculation from CAN/DM keepalive/send work.
- `TIM13`: 100 Hz low-frequency task tick.
- Enable `ProjectManager.RegisterCallBack=CAN,TIM,UART` so the existing library callback registration paths work.

### UserCode

- `app.cpp`:
  - Keep only `Device::init()`, `Chassis::init()`, `Protocol::init()`, timer startup, connection/initialization waiting, and `Chassis::enable()`.
- `device.*`:
  - Initialize gyro, CAN, and four DM S3519 wheel motors only.
  - Remove lift/grip/pressure/I2C/infrared devices.
- `chassis.*`:
  - Use `chassis::motion::Mecanum4` instead of the reference project's `IndLiftMecanum4`.
  - Keep EKF initialization, standalone localization/control initialization, Lidar updates, 1 kHz updates, and 100 Hz updates.
- `chassis/Config.hpp`:
  - Keep chassis wheel radius, wheelbase/track geometry, Master/Slave control parameters, and EKF noise parameters.
  - Delete lift/action-related configuration.
- `project_parts.hpp`:
  - Simplify to chassis capability switches: wheel chassis, gyro, PC localization, PC control, Slave control, and upper-host identify initialization.

### DM S3519 Motor Configuration

- Use `DMMotor::Mode::Vel`.
- Drive velocity commands through `MotorVelController`.
- Define `pos_max_rad`, `vel_max_rad`, and `tor_max` as centralized S3519 config constants.
- Mark those constants as values that must match the Damiao upper-tool driver configuration before hardware bring-up.
- Use a global unified DM `master_id`.
- Initialize filters through `DMMotor::CAN_FilterInit(hcan, bank, master_id)`.
- Register CAN callback through `DMMotor::CANBaseReceiveCallback`.

### Master / Slave Control Interface

- `Chassis::master_ctrl` uses `chassis::controller::Master`.
- `Chassis::slave_ctrl` uses `chassis::controller::Slave<N>`.
- Master commands switch to Master mode.
- Slave trajectory commands switch to Slave mode.
- When switching modes, stop/clear the previous controller before activating the next mode so stale trajectory points cannot continue taking effect.

### Serial Protocol

- Keep frame header `AA BB`, CRC16-Modbus, big-endian field encoding, 230400 baud, and 10 ms feedback period.
- Keep these commands:
  - `0x01 Ping`
  - `0x10 StopChassis`
  - `0x12 SlavePushChassisTrajectory`
  - `0x13 SetMasterChassisTargetCurrentState`
  - `0x14 SetMasterChassisTargetPreviousCurve`
  - `0x15 SetMasterChassisVelocity`
  - `0x21 LidarPosture`
- Remove non-chassis command handling from the build:
  - `SetChassisHeight`
  - `Grip`
  - `Step`
  - `KFS`
  - `TakeSpear`
  - other grip/lift/action commands from the reference project.
- Feedback frame is a new pure-chassis frame with:
  - timestamp
  - `x`
  - `y`
  - `yaw`
  - `vx`
  - `vy`
  - `wz`
  - control mode/status
  - connection/status
  - CRC16

## Public Interfaces

### `Device`

- `Device::Sensor::gyro_yaw`
- `Device::Motor::wheel[4]` has type `motors::DMMotor*`
- `Device::init()`
- `Device::update_1kHz()` handles DM periodic keepalive / required sends and no longer sends DJI iq commands.

### `Chassis`

- `Chassis::motion` has type `chassis::motion::Mecanum4*`
- `Chassis::loc`, `loc_ekf`, and `loc_encoder`
- `Chassis::master_ctrl` and `slave_ctrl`, or an equivalent current-controller plus mode-management interface
- `Chassis::initLocCtrl(init_posture)`
- `Chassis::initStandaloneLocCtrl()`
- `Chassis::updateLidar(posture, ticks)`
- `Chassis::update_1kHz()`
- `Chassis::update_100Hz()`
- `Chassis::enable()`

### `Protocol`

- `Protocol::PCProtocol`
- `Protocol::CommandHandler::enqueueFrame`
- `Protocol::Feedback::startTask`
- `Protocol::clock()`
- `Protocol::isPcLocalizationConnected()`
- `Protocol::isUpperHostIdentified()`

### `config::uart`

- `AuxControllerHost = &huart1`
- `SensorGyroYaw = &huart2`
- `UpperHost = &huart3`

## Out of Scope

- Grip, lift, suction, infrared, pressure, connection table, step actions, KFS, spear actions, or other non-chassis application features.
- Compatibility with old non-chassis feedback fields. This task defaults to a new pure chassis feedback frame rather than old frame length with zero-filled height fields.
- Hardware sign tuning beyond the software bring-up defaults.

## Assumptions

- The target board can use the reference project's CAN/UART/TIM/DMA peripheral layout. If current board pins differ, use the actual board pinout rather than mechanically copying the reference `.ioc`.
- The chassis is an ordinary four-wheel mecanum chassis without the reference project's custom lift structure.
- S3519 uses internal driver velocity mode (`DMMotor::Mode::Vel`) rather than MIT torque control.
- The DM S3519 protocol range constants are placeholders until verified against the Damiao upper-tool motor configuration.
- EKF uses gyro by default; `JustEncoder` may remain as a debug fallback.

## Acceptance Criteria

- Firmware config/build includes generated `can.h`, `usart.h`, `dma.h`, and `tim.h` used by linked sources.
- Static search shows no active references remain for `IndLiftMecanum4`, `LiftSide`, `Grip`, `Suction`, `Infrared`, `Connection`, or step-action features in compiled user/application code.
- `Mecanum4` is constructed with four non-null `MotorVelController` pointers backed by DM S3519 motors.
- `Mecanum4::enable()` can acquire controllers and reset wheel angle through the configured motor controllers.
- Master/Slave mode switching clears stale commands.
- 1 kHz update order is localization/EKF update, controller update, mecanum wheel controller update, then DM keepalive/send path.
- Protocol decode rejects bad CRC frames and handles the retained command set with correct scale factors and source restrictions for `LidarPosture`.
- Feedback frame length and field order match the pure chassis protocol.
- Project CMake configure/build succeeds, or remaining failures are documented as environment/toolchain blockers rather than source-level omissions.

## Hardware Bring-Up Checklist

- CAN1 sees S3519 `0x09` and `0x0A`.
- CAN2 sees S3519 `0x0B` and `0x0C`.
- DM `master_id` matches the motor feedback target ID and is shared by all DM motors.
- S3519 `pos_max_rad`, `vel_max_rad`, and `tor_max` match Damiao upper-tool configuration.
- Gyro yaw updates over USART2 before EKF initialization.
- Upper host receives periodic feedback over USART3 and can send `StopChassis`.
- With wheels off ground, `SetMasterChassisVelocity` signs match expected mecanum wheel directions.
