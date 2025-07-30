#pragma once

#include <cstdint>
#include <cstddef>

namespace bmi270
{
    // BMI270 configuration file (extracted from Bosch reference implementation)
    // This is the essential configuration data that must be uploaded to the BMI270
    // after soft reset to enable advanced features and proper operation
    extern const uint8_t bmi270_config_file[];
    extern const size_t bmi270_config_file_size;

    // Configuration file size constant
    constexpr size_t BMI270_CONFIG_FILE_SIZE = 8192; // Maximum size from reference
}
