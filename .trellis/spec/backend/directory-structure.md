# Directory Structure

> How firmware code is organized in this project.

---

## Overview

This repository is an STM32CubeMX CMake firmware project for a pure mecanum
chassis lower controller. Keep application-specific code under `UserCode/`.
Treat CubeMX outputs as generated and WTR modules as external package code unless
the task explicitly targets those packages.

---

## Directory Layout

```text
.
|-- chassis.ioc                 # CubeMX peripheral source of truth
|-- CMakeLists.txt              # Project CMake entry, glob-adds UserCode/*
|-- CMakePresets.json           # Debug preset used for normal verification
|-- Core/                       # CubeMX generated app/HAL glue
|-- Drivers/                    # CubeMX generated STM32 HAL/CMSIS drivers
|-- Middlewares/                # CubeMX generated middleware, FreeRTOS here
|-- startup_stm32f407xx.s       # Generated startup file
|-- STM32F407XX_FLASH.ld        # Linker script
|-- cmake/stm32cubemx/          # Generated CubeMX CMake integration
|-- cmake/wtr_modules.cmake     # In-tree WTR package registration contract
|-- Modules/                    # Local copies of WTR-managed reusable packages
|-- UserCode/                   # Active firmware application code
`-- wtrproject.toml             # WTR/cpkg dependency manifest
```

---

## UserCode Organization

- `UserCode/app.cpp`: application startup and timer callbacks. Keep the 1 kHz
  split update path easy to audit.
- `UserCode/device.*`: board-level sensor, CAN, UART, and motor object
  initialization. Hardware IDs and UART aliases live close to device wiring.
- `UserCode/chassis/`: chassis orchestration and project-specific chassis
  parameters. It wires module controllers together; it should not reimplement
  reusable chassis math that exists in `Modules/ChassisController/`.
- `UserCode/protocol/`: upper-host UART protocol decoding and command dispatch.
  Keep wire-format constants in `PCCommandDef.hpp`, frame structs in
  `PCProtocol.hpp`, decoding in `PCProtocol.cpp`, and behavior in
  `PCCommandHandler.cpp`.
- `UserCode/project_parts.hpp`: compile-time feature switches. Prefer
  `if constexpr` around optional firmware paths so disabled features compile out.
- `UserCode/system.hpp`: small global initialization gates shared across modules.
- `UserCode/sync/Clock.hpp`: project-local time alignment helper.

---

## Generated And Managed Code Boundaries

- `Core/`, `Drivers/`, `Middlewares/`, startup files, linker script, and
  `cmake/stm32cubemx/` are CubeMX outputs. Update `chassis.ioc` and run
  `stm32tool generate` when changing peripherals. Hand-edit generated output
  only as a documented fallback when generation is unavailable.
- `Modules/` contains WTR-managed package sources. Prefer using package APIs from
  `UserCode/`. Edit a module only when the task is explicitly about reusable
  driver/controller behavior.
- Keep `wtrproject.toml` and `cmake/wtr_modules.cmake` aligned. In this sandbox,
  local module copies are registered through `cmake/wtr_modules.cmake`.

---

## Naming Conventions

- C++ files in `UserCode/` use `.hpp` and `.cpp`.
- Types use `PascalCase`; namespaces, functions, variables, and constants usually
  follow the existing local style in the touched file.
- Project-specific aliases are allowed when they clarify module types, for
  example `Chassis::SlaveController`.
- Hardware wiring names should describe the logical signal, not only the UART
  number, for example `config::uart::UpperHostControl`.

---

## Examples

- `UserCode/device.cpp`: hardware setup is separated into small local init
  functions and guarded with `ProjectParts` switches.
- `UserCode/chassis/chassis.cpp`: application orchestration owns object
  lifetime and delegates motion/control math to WTR chassis modules.
- `UserCode/protocol/PCProtocol.cpp`: UART frame decoding is separate from
  command effects.
