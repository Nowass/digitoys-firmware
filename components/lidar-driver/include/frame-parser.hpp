#pragma once

#include <vector>
#include <cstdint>
#include <optional>

struct PointData
{
    float angle;        // degrees
    uint16_t distance;  // mm
    uint8_t confidence; // 0–255
};

struct Frame
{
    std::vector<PointData> points;
    float start_angle;
    float end_angle;
    uint16_t rpm;
    uint16_t timestamp;
};

class FrameParser
{
public:
    std::optional<Frame> parseFrame(const uint8_t *data, size_t len);
    static constexpr size_t FRAME_LEN = 47;

private:
    uint8_t calcCRC(const uint8_t *data, size_t len);
};
