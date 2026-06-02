# Database Guidelines

> Persistent storage applicability for this firmware project.

---

## Overview

This chassis firmware currently has no database, ORM, filesystem persistence, or
flash-backed application storage layer. Do not introduce database abstractions,
migration systems, or host-style repositories for normal firmware changes.

---

## Current State

- Runtime state is held in RAM through module instances and small global handles
  such as `Device::Motor::wheel`, `Chassis::loc`, and protocol receive buffers.
- Hardware/peripheral configuration is stored in `chassis.ioc` and generated
  CubeMX files.
- Build-time dependency state is stored in `wtrproject.toml` and
  `cmake/wtr_modules.cmake`.

---

## If Persistent Storage Is Added Later

- Treat flash/EEPROM writes as an embedded subsystem, not as a conventional
  database layer.
- Define record layout, versioning, wear-leveling, CRC/checksum behavior, and
  power-loss behavior before writing code.
- Keep persistent format constants in one owner module and document the wire or
  storage contract in `.trellis/spec/backend/embedded-firmware-contracts.md`.

---

## Forbidden Patterns

- Do not add generic database libraries or host-only persistence dependencies.
- Do not write calibration or runtime state to flash from ISR/timer callbacks.
- Do not store protocol frame bytes as an implicit persistent contract. Document
  the format explicitly instead.
