#pragma once

#include <cstdint>
#include <vector>
#include <optional>

namespace lidar {

struct Point {
    float angle_deg;
    uint16_t distance_mm;
    uint8_t confidence;
};

struct Frame {
    std::vector<Point> points;
    std::optional<uint16_t> rpm;
};

class FrameParser {
public:
    FrameParser();

    // Feed raw data; returns any complete frames parsed
    std::vector<Frame> parse(const uint8_t* data, size_t length);

private:
    std::vector<uint8_t> buffer;

    static constexpr uint8_t HEADER_1 = 0x54;
    static constexpr uint8_t HEADER_2 = 0x2C;
    static constexpr size_t FRAME_SIZE = 47;
    static constexpr size_t POINT_COUNT = 12;

    bool findNextFrame(size_t& index);
    bool validateChecksum(const uint8_t* frame);
    Frame parseFrame(const uint8_t* frame);
};

} // namespace lidar
