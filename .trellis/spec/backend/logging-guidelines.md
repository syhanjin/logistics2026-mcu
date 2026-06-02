# Logging Guidelines

> Firmware observability constraints for this project.

---

## Overview

The active firmware has no general logging framework and should not add
host-style structured logging in normal control code. Observability is currently
through build output, debugger inspection, watchdog/connection state, and
hardware bring-up checks.

---

## Current Observability Mechanisms

- CMake/linker output reports memory use after a successful build.
- Protocol connection state is represented by watchdog-backed functions such as
  `Protocol::isPcLocalizationConnected()`.
- Initialization gates live in `System::Init`.
- Hardware verification is done through CAN/UART bring-up checks and debugger
  inspection.

---

## What To Avoid

- Do not add `printf`, semihosting, dynamic string formatting, or blocking UART
  logging to 1 kHz control paths, interrupt callbacks, or protocol decode paths.
- Do not reuse control UARTs for ad hoc logs unless the task explicitly defines a
  debug protocol and verifies timing impact.
- Do not log raw host frames as a substitute for CRC validation and documented
  contracts.

---

## Acceptable Debug Hooks

- Temporary local debugger variables are acceptable during bring-up if removed or
  clearly scoped before completion.
- Small counters or state flags may be added when they are part of the firmware
  behavior contract.
- If a real telemetry protocol is added, document its frame format and timing in
  `embedded-firmware-contracts.md`.
