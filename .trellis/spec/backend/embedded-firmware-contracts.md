# Embedded Firmware Contracts

## Scenario: Pure Mecanum Chassis Firmware Port

### 1. Scope / Trigger

- Trigger: changes to the STM32 lower-controller firmware that affect chassis motion, upper-host serial protocol, CubeMX peripherals, WTR module wiring, or generated HAL integration.
- Scope: the `chassis` project as a pure four-wheel mecanum chassis controller. Non-chassis application features such as grip, lift, suction, infrared, connection-table actions, step actions, KFS, and spear actions must not be compiled into active `UserCode`.
- Generated-code rule: keep `.ioc` as the source of peripheral truth and regenerate with `stm32tool generate` after changing CubeMX settings. If the environment blocks generation, generated files under `Core/` and `Drivers/` may be aligned manually, but the final check must still build from those files.

### 2. Signatures

- UART handles:
  - `config::uart::AuxControllerHost = &huart1`
  - `config::uart::SensorGyroYaw = &huart2`
  - `config::uart::UpperHost = &huart3`
- Device handles:
  - `Device::Sensor::gyro_yaw`: `sensors::gyro::HWT101CT*`
  - `Device::Motor::wheel[4]`: `motors::DMMotor*`
- Chassis handles:
  - `Chassis::motion`: `chassis::motion::Mecanum4*`
  - `Chassis::loc`, `loc_ekf`, `loc_encoder`
  - `Chassis::master_ctrl`: `chassis::controller::Master*`
  - `Chassis::slave_ctrl`: `chassis::controller::Slave<64>*`
- Protocol frame:
  - Header: `AA BB`
  - Command payload length: `1 + 12 + 4 + 2 = 19`
  - Full command frame length: `21`
  - Command field order: `cmd u8 | data[12] | tx_timestamp u32 | crc16 u16`
- Feedback frame:
  - Header: `AA BB`
  - Payload length: `22`
  - Full feedback frame length: `24`
  - Payload field order: `timestamp u32 | x i16*2000 | y i16*2000 | yaw i16*100 | vx i16*2000 | vy i16*2000 | wz i16*100 | control u16 | connection u16 | crc16 u16`

### 3. Contracts

- CRC is CRC16-Modbus over payload bytes only, excluding the two-byte header and excluding the CRC field itself.
- Integer fields in upper-host protocol frames are big-endian.
- Retained commands are:
  - `0x01 Ping`
  - `0x10 StopChassis`
  - `0x12 SlavePushChassisTrajectory`
  - `0x13 SetMasterChassisTargetCurrentState`
  - `0x14 SetMasterChassisTargetPreviousCurve`
  - `0x15 SetMasterChassisVelocity`
  - `0x21 LidarPosture`
- `LidarPosture` may only be consumed from the main upper-host protocol, after the upper-host clock is stable, and after the gyro is connected for initial posture construction.
- DM S3519 wheel mapping is fixed until hardware wiring changes:
  - `wheel[0]` front-right: CAN1, `id0 = 0x09`
  - `wheel[1]` front-left: CAN1, `id0 = 0x0A`
  - `wheel[2]` rear-left: CAN2, `id0 = 0x0B`
  - `wheel[3]` rear-right: CAN2, `id0 = 0x0C`
- DM motors use `DMMotor::Mode::Vel`; the `pos_max_rad`, `vel_max_rad`, and `tor_max` constants must match the Damiao upper-tool driver configuration before hardware bring-up.
- The 1 kHz update order is localization update, controller update, `Mecanum4::update()`, then the half-period device/CAN keepalive path.
- Master/Slave mode switching must stop and release or clear the previous controller before the new controller receives target data.
- WTR module wiring must keep `wtrproject.toml` and `cmake/wtr_modules.cmake` aligned. In offline/local-copy workflows where `cpkg sync` cannot register copied module directories as submodules, `cmake/wtr_modules.cmake` is the build contract.

### 4. Validation & Error Matrix

- Bad CRC -> `PCProtocol::decode()` returns `false`; no command is enqueued.
- Unknown or removed command -> no active behavior should run; do not retain non-chassis switch cases in pure chassis `UserCode`.
- `LidarPosture` from auxiliary protocol -> ignore.
- `LidarPosture` while clock is unstable -> ignore.
- First `LidarPosture` before gyro connection -> ignore and keep initialization gate closed.
- Missing wheel motor pointer during chassis init -> `Error_Handler()`.
- `Chassis::enable()` before `master_ctrl` exists or can enable -> `Error_Handler()`.
- Missing generated `can/usart/dma/tim` sources or missing HAL CAN/UART drivers -> CMake build failure.

### 5. Good / Base / Bad Cases

- Good: main upper host sends several valid timestamped frames, clock stabilizes, first valid `LidarPosture` initializes EKF with gyro yaw offset, then Master velocity and target commands switch to Master and update wheel velocity references.
- Base: no external localization is enabled; `initStandaloneLocCtrl()` constructs local localization/control from `{0, 0, 0}`.
- Bad: a Slave trajectory point arrives after Master mode activity without clearing previous state. Correct behavior is `switchToSlave()` stopping/releasing Master, clearing the Slave trajectory buffer, acquiring Slave control, and then pushing the point.

### 6. Tests Required

- Build checks:
  - `cmake --preset Debug`
  - `cmake --build --preset Debug --clean-first`
- Static checks:
  - Generated files exist: `Core/Inc/{can,usart,dma,tim}.h` and `Core/Src/{can,usart,dma,tim}.c`.
  - HAL CAN/UART driver files exist under `Drivers/STM32F4xx_HAL_Driver/`.
  - Search active firmware paths for removed non-chassis terms: `IndLiftMecanum4`, `LiftSide`, `Grip`, `Suction`, `Infrared`, `Connection`, step actions, `TakeSpear`, `KFS`, `SetChassisHeight`, `DJIMotor`.
- Protocol checks:
  - Valid CRC frame enqueues one command.
  - Invalid CRC frame is rejected.
  - Master target/velocity and Slave trajectory decode scale factors exactly.
  - `LidarPosture` source and clock-stability restrictions are enforced.
  - Feedback length and field order match the contract above.
- Hardware bring-up checks:
  - CAN1 receives DM S3519 `0x09/0x0A`; CAN2 receives `0x0B/0x0C`.
  - `DMFeedbackMasterId` matches the configured motor feedback target ID.
  - HWT101CT yaw updates over USART2 before EKF initialization.

### 7. Wrong vs Correct

#### Wrong

```cpp
case PCCommand::SetChassisHeight:
case PCCommand::SetGripPose:
case PCCommand::StepUp200:
    // Pure chassis firmware should not keep these active command cases.
    break;
```

#### Correct

```cpp
case PCCommand::SetMasterChassisVelocity:
    if (!Chassis::switchToMaster() || Chassis::master_ctrl == nullptr)
        break;
    Chassis::master_ctrl->setVelocityInBody(read_velocity(frame.data, 0), false);
    break;
```

#### Wrong

```cpp
// Changing generated files only.
// The next CubeMX generation will erase the peripheral fix.
```

#### Correct

```text
Update chassis.ioc -> run stm32tool generate -> verify CMake includes generated sources and HAL drivers -> clean build.
```
