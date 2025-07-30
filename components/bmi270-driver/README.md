# BMI270 Driver Component

This component provides a C++ driver for the BMI270 6-axis inertial measurement unit (IMU) sensor from Bosch.

## Features

- Modern C++ implementation with RAII principles
- Generic I2C HAL (Hardware Abstraction Layer) for safe I2C communication
- Step-by-step implementation approach for reliability
- Comprehensive error handling and logging
- Support for both I2C addresses (0x68 and 0x69)
- Thread-safe operations

## Current Implementation Status

### ✅ Step 1: Basic Device Initialization (COMPLETED)

- **I2C HAL**: Generic, safe I2C communication layer
- **Device Detection**: Automatic device presence verification
- **Chip ID Verification**: Confirms BMI270 sensor (0x24)
- **Soft Reset**: Clean device state initialization
- **Error Checking**: Basic error register monitoring

### 🚧 Step 2: Configuration & Advanced Init (PLANNED)

- Configuration file upload (~8KB binary blob)
- Advanced sensor initialization sequence
- Power management configuration

### 🚧 Step 3: Sensor Data Reading (PLANNED)

- Accelerometer data reading (±2g/4g/8g/16g ranges)
- Gyroscope data reading (±125°/s to ±2000°/s ranges)
- Temperature sensor reading
- Sensor time synchronization

### 🚧 Step 4: Advanced Features (PLANNED)

- FIFO buffer management
- Interrupt handling (motion detection, data ready)
- Motion features (any-motion, no-motion, significant motion)
- Step counter and activity recognition

## Usage Example (Step 1)

```cpp
#include "bmi270.hpp"
#include "I2CConfig.hpp"

// Configure I2C for BMI270
bmi270::I2CConfig config;
config.port = I2C_NUM_0;
config.sdaPin = GPIO_NUM_4;
config.sclPin = GPIO_NUM_5;
config.clockSpeed = 400000;  // 400kHz
config.slaveAddr = 0x68;     // BMI270 default address

// Initialize BMI270 sensor
bmi270::BMI270 sensor;
if (sensor.init(config) == ESP_OK) {
    ESP_LOGI("APP", "BMI270 initialized successfully!");
    
    // Verify chip ID
    uint8_t chipId;
    if (sensor.readChipId(chipId) == ESP_OK) {
        ESP_LOGI("APP", "Chip ID: 0x%02X", chipId);
    }
    
    // Check status
    uint8_t status;
    if (sensor.readStatus(status) == ESP_OK) {
        ESP_LOGI("APP", "Status: 0x%02X", status);
    }
} else {
    ESP_LOGE("APP", "BMI270 initialization failed");
}
```

## Hardware Setup

Connect the BMI270 sensor:
- **SDA** → GPIO pin (configurable, e.g., GPIO4)
- **SCL** → GPIO pin (configurable, e.g., GPIO5)  
- **VCC** → 3.3V
- **GND** → Ground
- **Pull-up resistors** (4.7kΩ) on SDA and SCL lines (if not built into module)

The BMI270 can operate at either I2C address:
- **0x68** (default, SDO pin low or floating)
- **0x69** (alternative, SDO pin high)

## Configuration

### I2CConfig Structure

- `port`: I2C port number (I2C_NUM_0 or I2C_NUM_1)
- `sdaPin`: GPIO pin for SDA line
- `sclPin`: GPIO pin for SCL line  
- `clockSpeed`: I2C clock frequency in Hz (default: 400kHz)
- `pullupEnable`: Enable internal pull-up resistors (default: true)
- `timeoutMs`: Communication timeout in milliseconds (default: 1000ms)
- `slaveAddr`: I2C slave address (0x68 or 0x69 for BMI270)

## Implementation Notes

This driver is designed following the official Bosch BMI270 Sensor API as reference. The implementation is broken down into manageable steps:

1. **Step 1** establishes basic I2C communication and device identification
2. Future steps will add configuration file upload and sensor data reading
3. Advanced features like FIFO and interrupts will be implemented last

Each step is thoroughly tested before moving to the next, ensuring a reliable and maintainable driver.

## Dependencies

- ESP-IDF driver component  
- C++20 standard library features (`std::span`, `std::byte`)
- FreeRTOS (for delays and task management)

## Thread Safety

The BMI270 driver is designed to be thread-safe for read operations, but initialization and configuration should be done from a single thread.
