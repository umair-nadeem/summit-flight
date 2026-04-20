# summit-flight

Modular drone flight control software written in modern C++ (C++20), targeting STM32.

The repository currently contains a working quadcopter project named **Aeromight**, implementing a complete IMU-based flight control stack including attitude/rate control, state estimation, RC input processing, motor control (DShot), and system health monitoring. The system runs on **FreeRTOS** and is designed with deterministic execution, static memory allocation, and clear separation between reusable modules and project-specific glue code.

The software architecture follows embedded safety-critical software practices: no dynamic allocation at runtime, explicit ownership and lifetimes, strongly typed interfaces, and testable components through dependency injection.

## Platform

- STM32F411 @ 100 MHz (currently supported)

## Technologies

- C++20
- CMake-based build system
- FreeRTOS
- GoogleTest / GoogleMock (native unit testing)

## Repository Structure

- `external/` — 3rd party libraries
- `modules/` — reusable abstract modules (control, estimation, RC decoding, etc.)
- `drivers/` — hardware-independent drivers/protocol implementations (DShot, CRSF, sensors)
- `projects/` — project-specific implementations (currently `aeromight`)
- `platform/` — platform bindings: stm (STM32/FreeRTOS) and native (linux environment)

## Status

Aeromight quadcopter flight controller is implemented and flight-tested. Development is ongoing with a focus on improving modularity, test coverage, and SITL support.
