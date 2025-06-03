#ifndef __LIPKG_H
#define __LIPKG_H

#include <chrono>

#include "pointdata.h"
#include "tofbf.h"
#include <mutex>

namespace ldlidar
{

    enum
    {
        PKG_HEADER = 0x54,
        PKG_VER_LEN = 0x2C,
        POINT_PER_PACK = 12,
    };

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

    class LiPkg
    {
    public:
        const int kPointFrequence = 4500;

        LiPkg();
        ~LiPkg();
        // set product type (belong to enum class LDType)
        void SetProductType(LDType type_number);
        // get sdk version number
        std::string GetSdkVersionNumber(void);
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
        // the number of errors in parser process of lidar data frame
        long GetErrorTimes(void);
        void CommReadCallback(const char *byte, size_t len);
        Points2D GetLaserScanData(void);

    private:
        LDType product_type_;
        std::string sdk_pack_version_;
        uint16_t timestamp_;
        double speed_;
        long error_times_;
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

#endif //__LIPKG_H