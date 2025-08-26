# 📦 Components Overview

This document provides a high-level summary of the core software components. Each section links to a dedicated file with class diagrams, public API descriptions, and usage examples.

---

## 🎛️ Control Task

The heart of the DigiToys firmware, implementing the main control logic that orchestrates:

- LiDAR-based obstacle detection with dynamic thresholds
- RC input processing and vehicle state management  
- Safety-critical speed control with multiple intervention levels
- System state management and telemetry integration

👉 [See full Control Task API and architecture details](./control-task.md)

---

## 🟢 LiDAR Driver

The LiDAR driver handles:

- UART-based communication with the LiDAR device
- Frame parsing (angle, distance, confidence)
- Proximity filtering
- LiDAR motor control

👉 [See full LiDAR API and class details](./lidar-driver.md)

---

## 🟡 ADAS PWM Driver

This module manages PWM input/output channels for:

- Capturing throttle/steering signals (via RMT)
- Replaying or overriding PWM output (via LEDC)
- Supporting ADAS-controlled overrides like braking

👉 [See full ADAS PWM API and class details](./adas-pwm-driver.md)

---

## 🔵 Monitor

Responsible for:

- Collecting FreeRTOS statistics (heap, CPU, tasks)
- Converting system metrics into dashboard-friendly JSON

👉 [See full Monitor API and class details](./monitor.md)

---

## 🟢 BMI270 Driver

An accelerometer driver built around the Bosch BMI270 sensor. It exposes the
`IAccelSensor` interface and uses an I2C helper for bus access. The driver can be
polled via `dataReady()` or extended to trigger a GPIO interrupt for minimal CPU
usage.

👉 [See full BMI270 API and class details](./bmi270-driver.md)

---

## 🔁 Runtime Integration

All components are orchestrated by `ControlTask` from `main.cpp`, which:

- Implements the main control loop running at 50ms intervals (20Hz)
- Reads LiDAR data for obstacle detection and distance measurement
- Processes RC input signals to determine vehicle state and driver intent
- Applies sophisticated safety algorithms with speed-dependent thresholds
- Triggers PWM overrides for emergency braking and speed control
- Reports comprehensive system stats to the web dashboard
- Manages state transitions and provides diagnostic logging

### 🔧 Supporting Infrastructure

**DigiToys Core** (`digitoys-core`):
- **Centralized Logging System**: Production-ready logging with simplified 2-parameter API
- **Component Base Classes**: Standardized component lifecycle management  
- **Configuration Framework**: Validation and factory pattern for component configs
- **Error Handling**: Unified error reporting and recovery mechanisms
- **Constants Management**: Centralized system constants and configurations

All components use the **centralized logging system** for unified debugging and monitoring:
```cpp
// Each component registers once
DIGITOYS_REGISTER_COMPONENT("ComponentName", "TAG");

// Clean 2-parameter logging everywhere  
DIGITOYS_LOGI("ComponentName", "Message with %d parameters", count);
```

The system uses a multi-task FreeRTOS architecture with priority-based scheduling to ensure real-time performance for safety-critical operations.

👉 [See complete architecture overview](./architecture-overview.md)  
👉 [See detailed FreeRTOS task documentation](./freertos-task-architecture.md)
