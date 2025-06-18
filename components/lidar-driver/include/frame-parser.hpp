#pragma once
/**
 * @file frame-parser.hpp
 * @brief Parses raw LiDAR packets into usable point data frames.
 */
#include <chrono>

#include "pointdata.hpp"
#include "near-distance-filter.hpp"
#include <mutex>

namespace lidar
{

    constexpr int PKG_HEADER = 0x54;
    constexpr int PKG_VER_LEN = 0x2C;
    constexpr int POINT_PER_PACK = 12;
    constexpr int kPointFrequence = 1000;

    typedef struct __attribute__((packed))
    {
        uint16_t distance;
        uint8_t intensity;
    } LidarPointStructDef;

    typedef struct __attribute__((packed))
    {
        uint8_t header;
        uint8_t ver_len;
        uint16_t speed;
        uint16_t start_angle;
        LidarPointStructDef point[POINT_PER_PACK];
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc8;
    } LiDARFrameTypeDef;

    /// Parses raw serial packets into calibrated scan data.
    class FramePraser
    {
    public:
        const int kPointFrequence = 4500;

        FramePraser();
        ~FramePraser();
        /// Get LiDAR spin speed in Hz
        double GetSpeed(void);
        /// Get raw rotation speed value reported by sensor
        uint16_t GetSpeedOrigin(void);
        /// Get timestamp of last packet in milliseconds
        uint16_t GetTimestamp(void);
        /// True if a complete frame has been assembled
        bool IsFrameReady(void);
        /// Reset frame ready flag
        void ResetFrameReady(void);
        /// Manually set frame ready flag
        void SetFrameReady(void);
        /// Feed raw bytes from UART
        void CommReadCallback(const char *byte, size_t len);
        /// Retrieve the most recent assembled scan data
        Points2D GetLaserScanData(void);

    private:
        uint16_t timestamp_;
        double speed_;
        bool is_frame_ready_;
        LiDARFrameTypeDef pkg_;
        Points2D frame_tmp_;
        Points2D laser_scan_data_;
        std::mutex mutex_lock1_;
        std::mutex mutex_lock2_;

        /// Parse a single incoming byte
        bool AnalysisOne(uint8_t byte);
        /// Parse a buffer of bytes
        bool Parse(const uint8_t *data, long len);
        /// Combine packets into a calibrated frame
        bool AssemblePacket();
        /// Replace stored scan data
        void SetLaserScanData(Points2D &src);
    };

} // namespace lidar
