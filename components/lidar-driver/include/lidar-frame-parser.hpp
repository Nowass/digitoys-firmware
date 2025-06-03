#ifndef __LIPKG_HPP
#define __LIPKG_HPP

#include <chrono>

#include "pointdata.hpp"
#include "near-distance-filter.hpp"
#include <mutex>

namespace lidar
{

    // enum
    // {
    //     PKG_HEADER = 0x54,
    //     PKG_VER_LEN = 0x2C,
    //     // POINT_PER_PACK = 12,
    // };

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

    class FramePraser
    {
    public:
        const int kPointFrequence = 4500;

        FramePraser();
        ~FramePraser();
        // get Lidar spin speed (Hz)
        double GetSpeed(void);
        // get lidar spind speed (degree per second) origin
        uint16_t GetSpeedOrigin(void);
        // get time stamp of the packet
        uint16_t GetTimestamp(void);
        // Get lidar data frame ready flag
        bool IsFrameReady(void);
        // Lidar data frame readiness flag reset
        void ResetFrameReady(void);
        void SetFrameReady(void);
        void CommReadCallback(const char *byte, size_t len);
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

        // parse single packet
        bool AnalysisOne(uint8_t byte);
        bool Parse(const uint8_t *data, long len);
        // combine stantard data into data frames and calibrate
        bool AssemblePacket();
        void SetLaserScanData(Points2D &src);
    };

} // namespace ldlidar

#endif //__LIPKG_HPP