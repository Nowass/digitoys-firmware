#include "frame-parser.hpp"
#include <cstring>
#include <cmath>

namespace lidar {

FrameParser::FrameParser() {
    buffer.reserve(256);
}

std::vector<Frame> FrameParser::parse(const uint8_t* data, size_t length) {
    std::vector<Frame> frames;
    buffer.insert(buffer.end(), data, data + length);

    size_t i = 0;
    while (buffer.size() - i >= FRAME_SIZE) {
        size_t frame_start = i;
        if (findNextFrame(frame_start)) {
            if (buffer.size() - frame_start < FRAME_SIZE)
                break;

            const uint8_t* frame_ptr = &buffer[frame_start];
            if (validateChecksum(frame_ptr)) {
                frames.push_back(parseFrame(frame_ptr));
                i = frame_start + FRAME_SIZE;
            } else {
                ++i;
            }
        } else {
            break;
        }
    }

    if (i > 0)
        buffer.erase(buffer.begin(), buffer.begin() + i);

    return frames;
}

bool FrameParser::findNextFrame(size_t& index) {
    for (; index + 1 < buffer.size(); ++index) {
        if (buffer[index] == HEADER_1 && buffer[index + 1] == HEADER_2)
            return true;
    }
    return false;
}

bool FrameParser::validateChecksum(const uint8_t* frame) {
    uint16_t sum = 0;
    for (int i = 0; i < FRAME_SIZE - 1; ++i) {
        sum += frame[i];
    }
    return (sum & 0xFF) == frame[FRAME_SIZE - 1];
}

Frame FrameParser::parseFrame(const uint8_t* frame) {
    Frame result;
    result.points.reserve(POINT_COUNT);

    uint16_t fsa = frame[2] | (frame[3] << 8);
    uint16_t lsa = frame[4] | (frame[5] << 8);

    float fsa_deg = (fsa >> 1) / 64.0f;
    float lsa_deg = (lsa >> 1) / 64.0f;

    float angle_diff = lsa_deg - fsa_deg;
    if (angle_diff < 0) angle_diff += 360.0f;

    for (int i = 0; i < POINT_COUNT; ++i) {
        size_t offset = 6 + i * 3;
        uint16_t distance = frame[offset] | (frame[offset + 1] << 8);
        uint8_t confidence = frame[offset + 2];

        float angle = fsa_deg + angle_diff * (i / static_cast<float>(POINT_COUNT - 1));
        if (angle >= 360.0f) angle -= 360.0f;

        result.points.push_back({ angle, distance, confidence });
    }

    result.rpm = static_cast<uint16_t>(frame[42] | (frame[43] << 8)) / 10;
    return result;
}

} // namespace lidar
