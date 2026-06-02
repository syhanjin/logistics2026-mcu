# Embedded Firmware Contracts

## Scenario: Pure Mecanum Chassis Firmware

### 1. Scope / Trigger

- Trigger: changes to STM32 lower-controller firmware that affect chassis
  motion, upper-host serial protocol, CubeMX peripherals, WTR module wiring, or
  generated HAL integration.
- Scope: this repository is a pure four-wheel mecanum chassis controller.
  Non-chassis application features such as grip, lift, suction, infrared,
  connection-table actions, step actions, KFS, and spear actions must not be
  compiled into active `UserCode` behavior.
- Generated-code rule: keep `chassis.ioc` as the peripheral source of truth and
  regenerate with `stm32tool generate` after changing CubeMX settings. If the
  environment blocks generation, generated files under `Core/` and `Drivers/`
  may be aligned manually, but the final check must still build from those
  files.

### 2. Signatures

- UART handles:
  - `config::uart::UpperHostControl = &huart1`
  - `config::uart::SensorGyroYaw = &huart2`
  - `config::uart::LidarPostureHost = &huart3`
- Upper-host UART baud:
  - `UpperHostControl`: `230400`
  - `LidarPostureHost`: `230400`
- Device handles:
  - `Device::Sensor::gyro_yaw`: `sensors::gyro::HWT101CT*`
  - `Device::Motor::wheel[4]`: `motors::DMMotor*`
- Chassis handles:
  - `Chassis::motion`: `chassis::motion::Mecanum4*`
  - `Chassis::loc`, `loc_ekf`, `loc_encoder`
  - `Chassis::slave_ctrl`: `chassis::controller::Slave<64>*`
- Control mode:
  - `Chassis::ControlMode::None`
  - `Chassis::ControlMode::Slave`

### 3. Protocol Contracts

Common frame rules:

- Header: `AA BB`
- CRC: CRC16-Modbus over payload bytes only, excluding the two-byte header and
  excluding the CRC field itself.
- Multi-byte integer fields are little-endian.
- Floating-point fields are IEEE-754 `float` serialized little-endian.
- Invalid CRC -> `decode()` returns `false`; no frame is enqueued.

Lidar posture frame on `LidarPostureHost`:

- Payload length: `3 * f32 + 2 * u32 + crc16 = 22`
- Full frame length: `24`
- Payload order:
  - `x f32`
  - `y f32`
  - `yaw f32`
  - `lidar_timestamp u32`
  - `tx_timestamp u32`
  - `crc16 u16`
- Behavior:
  - Align `Protocol::clock()` with `rx_timestamp` and `tx_timestamp` plus UART
    transition delay.
  - Ignore posture data until the clock is stable.
  - Feed the lidar posture watchdog only after the clock is stable.
  - Before the first posture initializes localization, require the gyro object to
    exist and be connected.
  - After initialization, convert `lidar_timestamp` through
    `global_clock_.pcTime2SelfTime()` before calling `Chassis::updateLidar()`.

Upper-host control frame on `UpperHostControl`:

- Payload length: `12 * f32 + 2 * u16 + u8 + crc16 = 55`
- Full frame length: `57`
- Payload order:
  - chassis posture target: `x f32 | y f32 | yaw f32`
  - chassis velocity feed-forward: `dx f32 | dy f32 | dyaw f32`
  - reserved host fields currently ignored by this chassis firmware:
    `h f32 | dh f32 | q1 f32 | q2 f32 | dq1 f32 | dq2 f32 | angle1 u16 |
    angle2 u16 | flags u8`
  - `crc16 u16`
- Behavior:
  - Valid control frames enqueue an `UpperHostControlFrame`.
  - The command handler consumes only `x`, `y`, `yaw`, `dx`, `dy`, and `dyaw`
    into a `SlaveController::TrajectoryPoint`.
  - Reserved host fields are decoded to preserve the external frame layout, but
    they must not drive non-chassis behavior in this firmware.

Removed/absent protocol behavior:

- There is no active `enum class PCCommand` command enum in `UserCode`.
- There is no active Master controller protocol path.
- There is no active feedback transmitter or identify-byte transmitter.
- Do not reintroduce removed command cases unless the external protocol and this
  spec are updated together.

### 4. Chassis And Device Contracts

- DM S3519 wheel mapping is fixed until hardware wiring changes:
  - `wheel[0]` front-right: CAN1, `id0 = 0x09`
  - `wheel[1]` front-left: CAN1, `id0 = 0x0A`
  - `wheel[2]` rear-left: CAN2, `id0 = 0x0B`
  - `wheel[3]` rear-right: CAN2, `id0 = 0x0C`
- DM motors use `DMMotor::Mode::Vel`; `pos_max_rad`, `vel_max_rad`, and
  `tor_max` must match the Damiao upper-tool driver configuration before
  hardware bring-up.
- `Device::update_1kHz()` pings wheel motors when wheel chassis is enabled.
- The 1 kHz update order is localization update, Slave trajectory/error update,
  `Mecanum4::update()`, then the half-period device/CAN keepalive path from the
  second timer callback.
- `Chassis::enable()` requires `slave_ctrl` to exist and enable successfully.
  Failure is fatal.
- `Chassis::pushSlaveTrajectoryPoint()` must acquire Slave control through
  `switchToSlave()` before pushing the point. `switchToSlave()` clears stale
  trajectory points before acquiring control.
- `Chassis::stop()` must stop Slave control and clear queued trajectory points.
- WTR module wiring must keep `wtrproject.toml` and
  `cmake/wtr_modules.cmake` aligned. In offline/local-copy workflows where
  `cpkg sync` cannot register copied module directories as submodules,
  `cmake/wtr_modules.cmake` is the build contract.

### 5. Validation & Error Matrix

- Bad CRC -> decode returns `false`; no frame is enqueued.
- Valid lidar frame while clock is unstable -> ignore posture behavior.
- First valid lidar posture before gyro connection -> ignore and keep
  initialization gate closed.
- Missing wheel motor pointer during chassis init -> `Error_Handler()`.
- `Chassis::enable()` before `slave_ctrl` exists or can enable ->
  `Error_Handler()`.
- Failed UART receive startup in `Protocol::init()` -> `Error_Handler()`.
- Missing generated `can/usart/dma/tim` sources or missing HAL CAN/UART drivers
  -> CMake build failure.
- Reserved upper-host control fields -> decoded but ignored by chassis behavior.

### 6. Good / Base / Bad Cases

- Good: lidar host sends several valid timestamped posture frames, the clock
  stabilizes, the first valid posture initializes EKF with the gyro yaw offset,
  then upper-host control frames push Slave trajectory points and wheel velocity
  references update through `Slave::trajectoryUpdate()` and `Slave::errorUpdate()`.
- Base: PC localization disabled -> `initStandaloneLocCtrl()` constructs local
  localization/control from `{0, 0, 0}`.
- Bad: a control frame with a valid header but bad CRC arrives. Correct behavior
  is decode returning `false` with no trajectory point pushed.
- Bad: a valid control frame arrives before Slave localization/control exists.
  Correct behavior is `pushSlaveTrajectoryPoint()` returning `false`.

### 7. Tests Required

Build checks:

- `cmake --preset Debug`
- `cmake --build --preset Debug --clean-first`

Static checks:

- Generated files exist: `Core/Inc/{can,usart,dma,tim}.h` and
  `Core/Src/{can,usart,dma,tim}.c`.
- HAL CAN/UART driver files exist under `Drivers/STM32F4xx_HAL_Driver/`.
- Search active firmware paths for stale protocol/controller terms:

```bash
rg "PCFeedback|enum class PCCommand|SetMaster|switchToMaster|master_ctrl|AuxControllerHost|config::uart::UpperHost\\b|NeedUpperHostIdentifyInit|upperHostIdentified|isUpperHostIdentified" UserCode
```

- Search active firmware paths for removed non-chassis terms:

```bash
rg "IndLiftMecanum4|LiftSide|Grip|Suction|Infrared|ConnectionTable|StepUp|StepDown|TakeSpear|KFS|SetChassisHeight|DJIMotor" UserCode
```

Protocol checks:

- Valid lidar CRC frame enqueues one lidar posture frame.
- Invalid lidar CRC frame is rejected.
- Valid upper-host control CRC frame enqueues one control frame.
- Invalid upper-host control CRC frame is rejected.
- Decode little-endian `float`, `u32`, and `u16` fields exactly.
- Lidar posture clock-stability and gyro-initialization restrictions are
  enforced.
- Control frame reserved fields do not trigger non-chassis behavior.

Hardware bring-up checks:

- CAN1 receives DM S3519 `0x09/0x0A`; CAN2 receives `0x0B/0x0C`.
- `DMFeedbackMasterId` matches the configured motor feedback target ID.
- HWT101CT yaw updates over USART2 before EKF initialization.

### 8. Wrong vs Correct

#### Wrong

```cpp
case PCCommand::SetMasterChassisVelocity:
    // Stale command-enum protocol path from the previous frame format.
    break;
```

#### Correct

```cpp
const Chassis::SlaveController::TrajectoryPoint point{
        .p_ref = { .x = frame.x, .y = frame.y, .yaw = frame.yaw },
        .v_ref = { .vx = frame.dx, .vy = frame.dy, .wz = frame.dyaw },
};
(void)Chassis::pushSlaveTrajectoryPoint(point);
```

#### Wrong

```cpp
// Changing generated files only.
// The next CubeMX generation will erase the peripheral fix.
```

#### Correct

```text
Update chassis.ioc -> run stm32tool generate -> verify CMake includes generated
sources and HAL drivers -> clean build.
```
