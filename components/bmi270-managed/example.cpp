#include "bmi270_managed.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "BMI270_EXAMPLE";

using namespace bmi270_managed;

void bmi270_task(void *pvParameters)
{
    // Configure I2C for BMI270
    I2CConfig config;
    config.port = 0; // I2C port 0
    config.sda_pin = GPIO_NUM_4;
    config.scl_pin = GPIO_NUM_5;
    config.clk_speed = 400000;    // 400kHz
    config.device_address = 0x68; // BMI270 default address

    // Create BMI270 instance
    BMI270 sensor;

    // Initialize sensor
    esp_err_t ret = sensor.init(config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BMI270 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "BMI270 initialized successfully");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BMI270 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "BMI270 initialized successfully");

    // Verify chip ID
    uint8_t chip_id;
    ret = sensor.read_chip_id(&chip_id);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "BMI270 Chip ID: 0x%02X", chip_id);
    }

    // Configure accelerometer: ±4g range, 100Hz ODR
    ret = sensor.configure_accel(BMI2_ACC_RANGE_4G, BMI2_ACC_ODR_100HZ);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));
    }

    // Configure gyroscope: ±1000°/s range, 100Hz ODR
    ret = sensor.configure_gyro(BMI2_GYR_RANGE_1000, BMI2_GYR_ODR_100HZ);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));
    }

    // Enable both sensors
    ret = sensor.enable_sensors(true, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "BMI270 configured and enabled. Starting data reading...");

    // Main sensor reading loop
    uint32_t sample_count = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 10Hz reading

    while (1)
    {
        SensorData data;
        ret = sensor.read_sensor_data(&data);

        if (ret == ESP_OK)
        {
            // Print header every 50 samples for readability
            if (sample_count % 50 == 0)
            {
                ESP_LOGI(TAG, "Sample | Accel X | Accel Y | Accel Z | Gyro X | Gyro Y | Gyro Z | Temp");
                ESP_LOGI(TAG, "-------|---------|---------|---------|--------|--------|--------|------");
            }

            ESP_LOGI(TAG, "%6lu | %7.2f | %7.2f | %7.2f | %6.2f | %6.2f | %6.2f | %4.1f",
                     sample_count,
                     data.accel_valid ? data.accel.x : 0.0f,
                     data.accel_valid ? data.accel.y : 0.0f,
                     data.accel_valid ? data.accel.z : 0.0f,
                     data.gyro_valid ? data.gyro.x : 0.0f,
                     data.gyro_valid ? data.gyro.y : 0.0f,
                     data.gyro_valid ? data.gyro.z : 0.0f,
                     data.temp_valid ? data.temperature : 0.0f);

            sample_count++;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read sensor data: %s (BMI2 error: %d)",
                     esp_err_to_name(ret), sensor.get_last_error());
        }

        // Wait for next sample
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "BMI270 Managed Component Example");
    ESP_LOGI(TAG, "Using official Bosch Sensortec BMI270 API");

    // Create BMI270 task
    BaseType_t task_created = xTaskCreate(
        bmi270_task,
        "bmi270_task",
        4096,
        NULL,
        5,
        NULL);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create BMI270 task");
    }
}
