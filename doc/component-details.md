# 📦 Components Overview

This document provides a high-level summary of the core software components. Each section links to a dedicated file with class diagrams, public API descriptions, and usage examples.

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

- Reads LiDAR data
- Triggers PWM overrides based on proximity
- Reports system stats to the web dashboard
