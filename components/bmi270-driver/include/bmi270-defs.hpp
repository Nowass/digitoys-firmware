#pragma once

#include "i2c-hal.hpp"
#include "I2CConfig.hpp"
#include <esp_err.h>
#include <cstdint>

namespace bmi270
{

    /**
     * @brief BMI270 register addresses (essential ones for basic functionality)
     */
    namespace registers
    {
        constexpr uint8_t CHIP_ID = 0x00;         // Chip ID register
        constexpr uint8_t ERR_REG = 0x02;         // Error register
        constexpr uint8_t STATUS = 0x03;          // Status register
        constexpr uint8_t ACC_DATA_X_LSB = 0x0C;  // Accelerometer X LSB
        constexpr uint8_t GYR_DATA_X_LSB = 0x12;  // Gyroscope X LSB
        constexpr uint8_t SENSORTIME_0 = 0x18;    // Sensor time LSB
        constexpr uint8_t EVENT = 0x1B;           // Event register
        constexpr uint8_t INT_STATUS_0 = 0x1C;    // Interrupt status 0
        constexpr uint8_t INT_STATUS_1 = 0x1D;    // Interrupt status 1
        constexpr uint8_t SC_OUT_0 = 0x1E;        // Step counter output LSB
        constexpr uint8_t ORIENT_ACT = 0x26;      // Orientation activity
        constexpr uint8_t INTERNAL_STATUS = 0x2A; // Internal status
        constexpr uint8_t TEMPERATURE_LSB = 0x22; // Temperature LSB
        constexpr uint8_t TEMPERATURE_MSB = 0x23; // Temperature MSB
        constexpr uint8_t FIFO_LENGTH_LSB = 0x24; // FIFO length LSB
        constexpr uint8_t FIFO_DATA = 0x26;       // FIFO data
        constexpr uint8_t FEAT_PAGE = 0x2F;       // Feature page
        constexpr uint8_t ACC_CONF = 0x40;        // Accelerometer configuration
        constexpr uint8_t ACC_RANGE = 0x41;       // Accelerometer range
        constexpr uint8_t GYR_CONF = 0x42;        // Gyroscope configuration
        constexpr uint8_t GYR_RANGE = 0x43;       // Gyroscope range
        constexpr uint8_t AUX_CONF = 0x44;        // Auxiliary configuration
        constexpr uint8_t FIFO_DOWNS = 0x45;      // FIFO downsampling
        constexpr uint8_t FIFO_WTM_0 = 0x46;      // FIFO watermark LSB
        constexpr uint8_t FIFO_WTM_1 = 0x47;      // FIFO watermark MSB
        constexpr uint8_t FIFO_CONFIG_0 = 0x48;   // FIFO configuration 0
        constexpr uint8_t FIFO_CONFIG_1 = 0x49;   // FIFO configuration 1
        constexpr uint8_t SATURATION = 0x4A;      // Saturation
        constexpr uint8_t AUX_DEV_ID = 0x4B;      // Auxiliary device ID
        constexpr uint8_t AUX_IF_CONF = 0x4C;     // Auxiliary interface configuration
        constexpr uint8_t AUX_RD_ADDR = 0x4D;     // Auxiliary read address
        constexpr uint8_t AUX_WR_ADDR = 0x4E;     // Auxiliary write address
        constexpr uint8_t AUX_WR_DATA = 0x4F;     // Auxiliary write data
        constexpr uint8_t ERR_REG_MSK = 0x52;     // Error register mask
        constexpr uint8_t INT1_IO_CTRL = 0x53;    // INT1 I/O control
        constexpr uint8_t INT2_IO_CTRL = 0x54;    // INT2 I/O control
        constexpr uint8_t INT_LATCH = 0x55;       // Interrupt latch
        constexpr uint8_t INT1_MAP_FEAT = 0x56;   // INT1 map feature
        constexpr uint8_t INT2_MAP_FEAT = 0x57;   // INT2 map feature
        constexpr uint8_t INT_MAP_DATA = 0x58;    // Interrupt map data
        constexpr uint8_t INIT_CTRL = 0x59;       // Initialization control
        constexpr uint8_t INIT_ADDR_0 = 0x5B;     // Initialization address LSB
        constexpr uint8_t INIT_ADDR_1 = 0x5C;     // Initialization address MSB
        constexpr uint8_t INIT_DATA = 0x5E;       // Initialization data
        constexpr uint8_t FEATURES_REG = 0x30;    // Features register
        constexpr uint8_t PWR_CONF = 0x7C;        // Power configuration
        constexpr uint8_t PWR_CTRL = 0x7D;        // Power control
        constexpr uint8_t CMD = 0x7E;             // Command register
    }

    /**
     * @brief BMI270 constants and values
     */
    namespace constants
    {
        constexpr uint8_t CHIP_ID_VALUE = 0x24;  // Expected chip ID value
        constexpr uint8_t SOFT_RESET_CMD = 0xB6; // Soft reset command
        constexpr uint8_t FIFO_FLUSH_CMD = 0xB0; // FIFO flush command
    }

    /**
     * @brief BMI270 error codes
     */
    enum class BMI270_Error : int8_t
    {
        OK = 0,
        NULL_PTR = -1,
        COM_FAIL = -2,
        DEV_NOT_FOUND = -3,
        OUT_OF_RANGE = -4,
        INVALID_CFG = -5,
        CONFIG_LOAD = -9,
        INVALID_SENSOR = -8
    };

    /**
     * @brief Convert BMI270 error to ESP error
     */
    constexpr esp_err_t toEspError(BMI270_Error error)
    {
        switch (error)
        {
        case BMI270_Error::OK:
            return ESP_OK;
        case BMI270_Error::NULL_PTR:
            return ESP_ERR_INVALID_ARG;
        case BMI270_Error::COM_FAIL:
            return ESP_FAIL;
        case BMI270_Error::DEV_NOT_FOUND:
            return ESP_ERR_NOT_FOUND;
        case BMI270_Error::OUT_OF_RANGE:
            return ESP_ERR_INVALID_ARG;
        case BMI270_Error::INVALID_CFG:
            return ESP_ERR_INVALID_ARG;
        case BMI270_Error::CONFIG_LOAD:
            return ESP_ERR_INVALID_STATE;
        case BMI270_Error::INVALID_SENSOR:
            return ESP_ERR_INVALID_ARG;
        default:
            return ESP_FAIL;
        }
    }

    /**
     * @brief Basic accelerometer data structure
     */
    struct AccelData
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        uint32_t sensorTime = 0;
    };

    /**
     * @brief Basic gyroscope data structure
     */
    struct GyroData
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        uint32_t sensorTime = 0;
    };

} // namespace bmi270
