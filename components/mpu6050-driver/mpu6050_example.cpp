#include "mpu6050_example.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdlib>

static const char *TAG = "MPU6050_EXAMPLE";

namespace MPU6050Example
{

    Example::Example()
        : i2c_(I2C::Config{
              .port = I2C_NUM_0,
              .sda_pin = SDA_PIN,
              .scl_pin = SCL_PIN,
              .frequency_hz = I2C_FREQ}),
          sensor_(i2c_, MPU6050::Config{
                            .accelRange = MPU6050::Config::ACCEL_RANGE_2G, // Reduce range for better precision
                            .gyroRange = MPU6050::Config::GYRO_RANGE_250,  // Reduce range for better precision
                            .dlpf = MPU6050::Config::DLPF_44HZ,
                            .sampleRateDiv = 19, // 50Hz with 1kHz internal rate
                            .filterAlpha = 0.2f,
                            .i2c_address = 0x68}),
          initialized_(false), gyroOffsetX_(0.0f), gyroOffsetY_(0.0f), gyroOffsetZ_(0.0f), accelOffsetX_(0.0f), accelOffsetY_(0.0f), accelOffsetZ_(0.0f), tempOffset_(0.0f), calibrated_(false), currentAcceleration_(0.0f), currentVelocity_(0.0f), rawAccelX_(0), lastUpdateTime_(0), stationaryTime_(0), stationaryAccelSum_(0.0f), stationaryCount_(0), impactDetected_(false), hardBrakingDetected_(false), peakAcceleration_(0.0f), impactTime_(0)
    {
        // Initialize last data to zero
        lastData_ = {};
    }

    esp_err_t Example::initialize()
    {
        ESP_LOGI(TAG, "Initializing MPU6050 example...");

        // Initialize I2C bus
        esp_err_t ret = i2c_.init();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
            return ret;
        }

        // Test I2C communication first
        ESP_LOGI(TAG, "Testing I2C communication...");

        // Initialize MPU6050 sensor
        ESP_LOGI(TAG, "Initializing MPU6050 sensor...");
        if (!sensor_.init())
        {
            ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor");
            return ESP_FAIL;
        }

        // Check if sensor is responding properly
        MPU6050::SensorData testData;
        if (!sensor_.readAll(testData))
        {
            ESP_LOGE(TAG, "Failed to read initial sensor data");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Initial sensor readings:");
        ESP_LOGI(TAG, "  Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²", testData.accelX, testData.accelY, testData.accelZ);
        ESP_LOGI(TAG, "  Gyro: X=%.2f, Y=%.2f, Z=%.2f deg/s", testData.gyroX, testData.gyroY, testData.gyroZ);
        ESP_LOGI(TAG, "  Temperature: %.2f °C", testData.temperature);

        initialized_ = true;
        ESP_LOGI(TAG, "MPU6050 example initialized successfully");
        return ESP_OK;
    }

    esp_err_t Example::self_test()
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "MPU6050 not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Performing MPU6050 self-test...");

        if (!sensor_.selfTest())
        {
            ESP_LOGE(TAG, "MPU6050 self-test failed");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "MPU6050 self-test passed");
        return ESP_OK;
    }

    esp_err_t Example::calibrate()
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "MPU6050 not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "🔧 CALIBRATING MPU6050 - Keep sensor PERFECTLY STILL for 5 seconds...");
        ESP_LOGI(TAG, "📊 Configuration: ±2G accel range, ±250°/s gyro range");
        ESP_LOGI(TAG, "⏱️  This process removes sensor bias/offset for accurate readings");

        // Collect calibration samples
        const int numSamples = 50; // 50 samples over 5 seconds
        float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
        float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
        float tempSum = 0;

        ESP_LOGI(TAG, "📈 Collecting %d bias calibration samples...", numSamples);

        for (int i = 0; i < numSamples; i++)
        {
            MPU6050::SensorData data;
            if (!sensor_.readAll(data))
            {
                ESP_LOGE(TAG, "❌ Failed to read sensor data during calibration");
                return ESP_FAIL;
            }

            // Log first few samples to show what we're measuring
            if (i < 3)
            {
                ESP_LOGI(TAG, "📋 Sample %d: Accel X=%.3f Y=%.3f Z=%.3f m/s²", i + 1, data.accelX, data.accelY, data.accelZ);
            }

            gyroSumX += data.gyroX;
            gyroSumY += data.gyroY;
            gyroSumZ += data.gyroZ;
            accelSumX += data.accelX;
            accelSumY += data.accelY;
            accelSumZ += data.accelZ;
            tempSum += data.temperature;

            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms between samples
        }

        // Calculate bias offsets (these will be subtracted from future readings)
        gyroOffsetX_ = gyroSumX / numSamples;
        gyroOffsetY_ = gyroSumY / numSamples;
        gyroOffsetZ_ = gyroSumZ / numSamples;

        // For accelerometer: X and Y should be ~0 when stationary
        // Z-axis will read ~9.81 m/s² (gravity) - we DON'T calibrate Z-axis
        accelOffsetX_ = accelSumX / numSamples;
        accelOffsetY_ = accelSumY / numSamples;
        accelOffsetZ_ = 0.0f; // Keep gravity reading intact

        // Temperature calibration to room temperature
        float avgTemp = tempSum / numSamples;
        tempOffset_ = avgTemp - 22.0f;

        calibrated_ = true;

        ESP_LOGI(TAG, "✅ CALIBRATION COMPLETED - Bias offsets calculated:");
        ESP_LOGI(TAG, "🎯 X-Axis Accel Bias: %.3f m/s² (will be removed)", accelOffsetX_);
        ESP_LOGI(TAG, "🎯 Y-Axis Accel Bias: %.3f m/s² (will be removed)", accelOffsetY_);
        ESP_LOGI(TAG, "🎯 Z-Axis Accel: %.3f m/s² (gravity - not calibrated)", accelSumZ / numSamples);
        ESP_LOGI(TAG, "🎯 Gyro Bias X/Y/Z: %.2f/%.2f/%.2f deg/s", gyroOffsetX_, gyroOffsetY_, gyroOffsetZ_);
        ESP_LOGI(TAG, "🎯 Temperature offset: %.2f °C", tempOffset_);
        ESP_LOGI(TAG, "🚀 Ready for accurate motion detection!");

        return ESP_OK;
    }

    esp_err_t Example::read_and_log_data()
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "MPU6050 not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        // Read all sensor data at once
        if (!sensor_.readAll(lastData_))
        {
            ESP_LOGE(TAG, "Failed to read sensor data");
            return ESP_FAIL;
        }

        // SKIP ALL CALIBRATION - Show truly raw values
        // if (calibrated_)
        // {
        //     applyCalibratedData(lastData_);
        // }

        // SKIP MOTION DETECTION - Focus on raw data only
        // updateMotionDetection(lastData_);

        // === HARDWARE DIAGNOSTIC - SHOW RAW ADC VALUES ===
        ESP_LOGI(TAG, "┌─────── HARDWARE DIAGNOSTIC ──────────────────┐");
        
        // Let's read the raw ADC values directly to diagnose the hardware
        int16_t rawAccelX, rawAccelY, rawAccelZ, rawTemp;
        if (sensor_.readRawADC(rawAccelX, rawAccelY, rawAccelZ, rawTemp))
        {
            // Calculate scale factor for ±2g range
            float accelScale = 9.81f / 16384.0f;
            
            ESP_LOGI(TAG, "│ RAW ADC X: %6d → %6.3f m/s²           │", rawAccelX, rawAccelX * accelScale);
            ESP_LOGI(TAG, "│ RAW ADC Y: %6d → %6.3f m/s²           │", rawAccelY, rawAccelY * accelScale);
            ESP_LOGI(TAG, "│ RAW ADC Z: %6d → %6.3f m/s²           │", rawAccelZ, rawAccelZ * accelScale);
            ESP_LOGI(TAG, "│ RAW TEMP:  %6d → %6.1f°C              │", rawTemp, (rawTemp / 340.0f) + 36.53f);
            ESP_LOGI(TAG, "│                                              │");
            ESP_LOGI(TAG, "│ Expected Z (gravity): ~16384 ADC → ~9.8 m/s² │");
            ESP_LOGI(TAG, "│ Expected X/Y (stationary): ~0 ADC → ~0 m/s²  │");
        }
        else
        {
            ESP_LOGE(TAG, "│ ❌ FAILED TO READ RAW SENSOR DATA!           │");
        }
        
        ESP_LOGI(TAG, "│                                              │");
        ESP_LOGI(TAG, "│ Driver processed values:                     │");
        ESP_LOGI(TAG, "│ PROC X: %6.3f m/s²  PROC Y: %6.3f m/s²     │", lastData_.accelX, lastData_.accelY);
        ESP_LOGI(TAG, "│ PROC Z: %6.3f m/s² (gravity reference)      │", lastData_.accelZ);
        ESP_LOGI(TAG, "│ Temperature: %6.1f°C                       │", lastData_.temperature);
        ESP_LOGI(TAG, "└─────────────────────────────────────────────┘");

        // Additional sensor data (less frequent logging)
        static int log_counter = 0;
        if (++log_counter >= 5) // Log every 5th reading (every ~250ms)
        {
            ESP_LOGI(TAG, "Full sensor: Accel(X:%.2f Y:%.2f Z:%.2f) Gyro(X:%.1f Y:%.1f Z:%.1f) Temp:%.1f°C",
                     lastData_.accelX, lastData_.accelY, lastData_.accelZ,
                     lastData_.gyroX, lastData_.gyroY, lastData_.gyroZ, lastData_.temperature);
            log_counter = 0;
        }

        return ESP_OK;
    }

    void Example::applyCalibratedData(MPU6050::SensorData &data)
    {
        // Apply gyroscope offset calibration
        data.gyroX -= gyroOffsetX_;
        data.gyroY -= gyroOffsetY_;
        data.gyroZ -= gyroOffsetZ_;

        // Apply accelerometer offset calibration (X and Y axes only)
        data.accelX -= accelOffsetX_;
        data.accelY -= accelOffsetY_;
        // Don't apply offset to Z-axis as it should read gravity

        // Apply temperature offset calibration
        data.temperature -= tempOffset_;
    }

    void Example::updateMotionDetection(const MPU6050::SensorData &data)
    {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Initialize timing on first call
        if (lastUpdateTime_ == 0)
        {
            lastUpdateTime_ = currentTime;
            currentAcceleration_ = data.accelX;  // Just store raw value, no filtering
            currentVelocity_ = 0.0f;
            stationaryTime_ = currentTime;
            return;
        }

        // Calculate time delta
        float deltaTime = (currentTime - lastUpdateTime_) / 1000.0f;
        if (deltaTime <= 0.0f)
            return;

        // Use raw data directly (no problematic filtering)
        currentAcceleration_ = data.accelX;

        // ADAPTIVE BIAS CORRECTION - Check if sensor appears stationary
        if (abs(currentAcceleration_) < STATIONARY_THRESHOLD)
        {
            // Accumulate readings during stationary period
            stationaryAccelSum_ += currentAcceleration_;
            stationaryCount_++;
            
            // If we've been stationary long enough, update bias
            if ((currentTime - stationaryTime_) > BIAS_UPDATE_TIME && stationaryCount_ >= BIAS_UPDATE_SAMPLES)
            {
                float avgAcceleration = stationaryAccelSum_ / stationaryCount_;
                
                // Update X-axis bias if drift is significant
                if (abs(avgAcceleration) > 0.05f)  // Only update if drift > 0.05 m/s²
                {
                    accelOffsetX_ += avgAcceleration;  // Adjust bias to compensate for drift
                    ESP_LOGW(TAG, "🔧 BIAS DRIFT CORRECTION: X-bias updated by %.3f m/s² (new total: %.3f)", 
                             avgAcceleration, accelOffsetX_);
                }
                
                // Reset stationary tracking
                stationaryTime_ = currentTime;
                stationaryAccelSum_ = 0.0f;
                stationaryCount_ = 0;
            }
        }
        else
        {
            // Motion detected - reset stationary tracking
            stationaryTime_ = currentTime;
            stationaryAccelSum_ = 0.0f;
            stationaryCount_ = 0;
        }

        // SIMPLIFIED: Skip velocity integration for now - focus on raw data
        currentVelocity_ = 0.0f;  // Reset to avoid accumulated errors

        // Impact detection based on RAW acceleration magnitude
        float accelMagnitude = abs(currentAcceleration_);
        if (accelMagnitude > peakAcceleration_)
        {
            peakAcceleration_ = accelMagnitude;
        }

        // Impact detection - sudden high acceleration/deceleration  
        if (accelMagnitude > IMPACT_THRESHOLD)
        {
            if (!impactDetected_)
            {
                impactDetected_ = true;
                impactTime_ = currentTime;
                ESP_LOGW("AUTOMOTIVE", "🚨 COLLISION ALERT: %.2f m/s² (RAW)", accelMagnitude);
            }
        }
        else if (impactDetected_ && (currentTime - impactTime_) > 2000)
        {
            impactDetected_ = false;
            peakAcceleration_ = 0.0f;
        }

        // Hard braking detection - significant deceleration
        if (currentAcceleration_ < -HARD_BRAKE_THRESHOLD)
        {
            if (!hardBrakingDetected_)
            {
                hardBrakingDetected_ = true;
                ESP_LOGW("AUTOMOTIVE", "🛑 EMERGENCY BRAKE: %.2f m/s² deceleration (RAW)", -currentAcceleration_);
            }
        }
        else if (hardBrakingDetected_ && currentAcceleration_ > -2.0f)
        {
            hardBrakingDetected_ = false;
        }

        // Gradually decay peak acceleration
        peakAcceleration_ *= 0.995f;

        lastUpdateTime_ = currentTime;
    }

    void Example::resetImpactDetection()
    {
        impactDetected_ = false;
        hardBrakingDetected_ = false;
        peakAcceleration_ = 0.0f;
        currentAcceleration_ = 0.0f;
        currentVelocity_ = 0.0f;
        lastUpdateTime_ = 0;
        ESP_LOGI("AUTOMOTIVE", "Impact detection and velocity reset");
    }

    void Example::resetVelocity()
    {
        currentVelocity_ = 0.0f;
        ESP_LOGI("AUTOMOTIVE", "Velocity reset to zero");
    }

    void Example::logVelocityCalculation() const
    {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float deltaTime = (lastUpdateTime_ > 0) ? (currentTime - lastUpdateTime_) / 1000.0f : 0.0f;
        
        ESP_LOGI(TAG, "🧮 VELOCITY CALCULATION BREAKDOWN:");
        ESP_LOGI(TAG, "   📏 Current Acceleration: %.3f m/s²", currentAcceleration_);
        ESP_LOGI(TAG, "   ⏱️  Time Delta: %.3f seconds (FreeRTOS ticks)", deltaTime);
        ESP_LOGI(TAG, "   🔧 Formula: velocity += acceleration × deltaTime");
        ESP_LOGI(TAG, "   📊 Increment: %.3f += %.3f × %.3f = %.3f m/s", 
                 currentVelocity_ - (currentAcceleration_ * deltaTime), 
                 currentAcceleration_, deltaTime, currentVelocity_);
        ESP_LOGI(TAG, "   🚗 Final Speed: %.2f m/s = %.1f km/h", currentVelocity_, getCurrentSpeed());
        ESP_LOGI(TAG, "   📝 Update Rate: ~50ms (20Hz) from main task");
    }

    bool Example::getLatestData(MPU6050::SensorData &data)
    {
        if (!initialized_)
        {
            return false;
        }

        data = lastData_;
        return true;
    }

} // namespace MPU6050Example
