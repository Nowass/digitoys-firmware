# BMI270 Driver Component

This component provides a lightweight wrapper around Bosch's BMI270 inertial sensor for the ESP32-C6 using the ESP-IDF I2C driver. Only minimal register definitions are included; the official Bosch `BMI270_SensorAPI` should be linked to enable the full feature set.

## Usage

```
#include "BMI270.hpp"

bmi270::Config cfg{
    .port = I2C_NUM_0,
    .sda_pin = GPIO_NUM_4,
    .scl_pin = GPIO_NUM_5,
    .clk_speed_hz = 400000,
    .address = 0x68,
    .int_pin = GPIO_NUM_6,
    .accel_range = 0x00, // ±2g
    .odr_hz = 100
};

bmi270::BMI270 sensor(cfg);
ESP_ERROR_CHECK(sensor.init());
```

The driver uses the sensor FIFO and data-ready interrupt (if `int_pin` is set) to minimize CPU polling.
