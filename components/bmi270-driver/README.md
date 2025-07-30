# BMI270 Driver Component

This component provides drivers for the BMI270 6-axis inertial measurement unit (IMU) sensor from Bosch.

## Features

- Generic I2C HAL (Hardware Abstraction Layer) for safe I2C communication
- Modern C++ implementation with RAII principles
- Comprehensive error handling
- Support for both I2C addresses (0x68 and 0x69)
- Thread-safe operations

## Current Implementation

### I2C HAL (i2c-hal)

The I2C HAL provides a generic, safe interface for I2C communication that can be used with any I2C device, including the BMI270 sensor.

#### Key Features:
- **RAII Management**: Automatic resource cleanup in destructor
- **Type Safety**: Uses `std::span<std::byte>` for safe memory handling
- **Error Handling**: Comprehensive ESP-IDF error reporting
- **Device Detection**: Built-in ping functionality to verify device presence
- **Flexible Configuration**: Configurable I2C parameters (speed, pins, timeouts)

#### Usage Example:

```cpp
#include "i2c-hal.hpp"
#include "I2CConfig.hpp"

// Configure I2C
bmi270::I2CConfig config;
config.port = I2C_NUM_0;
config.sdaPin = GPIO_NUM_21;
config.sclPin = GPIO_NUM_22;
config.clockSpeed = 400000; // 400kHz
config.slaveAddr = 0x68;    // BMI270 default address

// Initialize I2C HAL
bmi270::I2C_HAL i2c;
if (i2c.init(config) != ESP_OK) {
    // Handle initialization error
    return;
}

// Check if device is present
if (i2c.ping() != ESP_OK) {
    // Device not found
    return;
}

// Read single byte from register 0x00 (chip ID)
uint8_t chipId;
if (i2c.readByte(0x00, chipId) == ESP_OK) {
    printf("Chip ID: 0x%02X\\n", chipId);
}

// Write single byte to register
if (i2c.writeByte(0x7E, 0x11) != ESP_OK) {
    // Handle write error
}

// Read multiple bytes
std::array<std::byte, 6> buffer;
if (i2c.read(0x0C, std::span<std::byte>(buffer)) == ESP_OK) {
    // Process acceleration data
}

// I2C HAL automatically deinitializes in destructor
```
