# Error Handling

> How errors are handled in this firmware project.

---

## Overview

This is bare-metal/FreeRTOS STM32 firmware. There is no exception-based error
model and no API error response layer. Runtime error handling is fail-fast for
configuration or hardware bring-up faults, and drop/ignore for invalid external
protocol input.

---

## Error Types

- Fatal setup/runtime invariant failures call `Error_Handler()`.
- Compile-time configuration violations use `static_assert`, for example feature
  dependencies in `UserCode/project_parts.hpp`.
- Development-only assumptions use `assert`, mainly around init-time hardware
  configuration such as expected UART baud rates.
- Recoverable protocol validation failures return `false` from decode/enqueue
  paths or are ignored by handlers.

---

## Required Patterns

- Use `Error_Handler()` when firmware cannot safely continue, such as missing
  wheel motor pointers, failed UART receive startup, or enabling chassis control
  before the required controller exists.
- Use CRC checks before enqueuing protocol frames. Invalid frames must not reach
  behavior handlers.
- Use `if constexpr (ProjectParts::...)` for optional features so disabled paths
  compile out.
- Keep ISR/UART callbacks small. They should delegate to protocol/module
  callbacks and avoid allocating or doing heavy behavior work.
- For ring-buffer enqueue failure, return `false` to the protocol layer; do not
  block inside decode.

---

## Forbidden Patterns

- Do not use C++ exceptions.
- Do not spin, delay, allocate repeatedly, or perform heavy control logic inside
  timer/UART/CAN callbacks.
- Do not silently proceed after failed hardware bring-up.
- Do not turn bad host input into chassis motion. Bad CRC, unstable clock, or
  missing initialization gates must prevent command effects.

---

## Examples

```cpp
if (!started)
    Error_Handler();
```

```cpp
const uint16_t crc = CRC16Modbus::calc(data, LidarPayloadLen - 2);
if (crc != crc_in_data)
    return false;
```
