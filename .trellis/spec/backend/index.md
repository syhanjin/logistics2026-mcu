# Backend Development Guidelines

> Best practices for backend development in this project.

---

## Overview

This repository is an STM32 C/C++ firmware project. Treat "backend" specs as the
authoritative firmware guidelines for `UserCode/`, CubeMX generated files, and
WTR-managed embedded modules.

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Module organization and file layout | Active |
| [Database Guidelines](./database-guidelines.md) | Persistent storage applicability | Active |
| [Error Handling](./error-handling.md) | Embedded error handling and fail-fast rules | Active |
| [Quality Guidelines](./quality-guidelines.md) | Code standards, forbidden patterns, validation | Active |
| [Logging Guidelines](./logging-guidelines.md) | Firmware observability constraints | Active |
| [Embedded Firmware Contracts](./embedded-firmware-contracts.md) | STM32 chassis firmware protocol, peripheral, and module contracts | Active |

---

## How to Fill These Guidelines

For each guideline file:

1. Document your project's **actual conventions** (not ideals)
2. Include **code examples** from your codebase
3. List **forbidden patterns** and why
4. Add **common mistakes** your team has made

The goal is to help AI assistants and new team members understand how YOUR project works.

---

**Language**: All documentation should be written in **English**.
