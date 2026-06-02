# Quality Guidelines

> Code quality standards for firmware development.

---

## Overview

Prioritize deterministic control behavior, clear hardware/protocol contracts, and
buildable STM32 output. Keep changes localized to `UserCode/` unless the task
explicitly targets CubeMX configuration or reusable WTR packages.

---

## Required Patterns

- Build with the repository preset before finishing:
  - `cmake --preset Debug`
  - `cmake --build --preset Debug --clean-first`
- Keep protocol parsing, buffering, and behavior separated:
  - constants in `PCCommandDef.hpp`
  - frame structs/classes in `PCProtocol.hpp`
  - decode/CRC/endian handling in `PCProtocol.cpp`
  - behavior in `PCCommandHandler.cpp`
- Use module APIs instead of duplicating chassis, motor, CRC, ring-buffer, or
  UART-sync logic already present under `Modules/`.
- Keep heap allocation to init-time object construction; do not allocate in
  periodic callbacks or decode/handler hot paths.
- Use explicit feature switches from `ProjectParts` for optional runtime
  capabilities.
- Preserve generated-code boundaries. Peripheral changes start at `chassis.ioc`.

---

## Forbidden Patterns

- Do not reintroduce non-chassis behavior into active `UserCode` for this pure
  chassis firmware.
- Do not keep stale protocol command cases or feedback code that no active frame
  format owns.
- Do not change protocol field order, endian, scale, CRC range, or frame length
  without updating `embedded-firmware-contracts.md`.
- Do not hand-edit generated CubeMX output as the only source of a peripheral
  change.
- Do not add blocking work to 1 kHz callbacks.

---

## Static Checks

For protocol/chassis changes, search active firmware paths for stale concepts:

```bash
rg "PCFeedback|enum class PCCommand|SetMaster|switchToMaster|master_ctrl|AuxControllerHost|config::uart::UpperHost\\b|NeedUpperHostIdentifyInit|upperHostIdentified|isUpperHostIdentified" UserCode
```

For pure chassis cleanup, search active firmware paths for removed non-chassis
features such as grip, lift, suction, step actions, KFS, and spear actions.

---

## Code Review Checklist

- Build passes with the Debug preset.
- `UserCode/` behavior matches `.trellis/spec/backend/embedded-firmware-contracts.md`.
- UART/CAN handles and motor IDs match physical wiring.
- Protocol decode validates CRC before enqueue.
- Optional features compile out through `ProjectParts`.
- Timer callbacks remain bounded and easy to audit.
