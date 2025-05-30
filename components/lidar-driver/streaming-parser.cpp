#include "streaming-parser.hpp"
#include <algorithm>

std::vector<Frame> StreamingParser::parseStream(const uint8_t *data, size_t len)
{
    std::vector<Frame> frames;

    // Append new data to internal buffer
    buffer.insert(buffer.end(), data, data + len);

    // Process as many frames as possible
    while (buffer.size() >= FrameParser::FRAME_LEN)
    {
        // Sync on frame header (0x54 0x2C)
        auto it = std::search(buffer.begin(), buffer.end(),
                              std::begin({0x54, 0x2C}),
                              std::end({0x54, 0x2C}));

        size_t offset = std::distance(buffer.begin(), it);

        if (offset + FrameParser::FRAME_LEN > buffer.size())
        {
            // Not enough data yet
            break;
        }

        // Try to parse frame from that location
        auto maybe_frame = frameParser.parseFrame(buffer.data() + offset, FrameParser::FRAME_LEN);

        if (maybe_frame.has_value())
        {
            frames.push_back(*maybe_frame);
            buffer.erase(buffer.begin(), buffer.begin() + offset + FrameParser::FRAME_LEN);
        }
        else
        {
            // Bad frame — discard 1 byte and try again
            buffer.erase(buffer.begin(), buffer.begin() + offset + 1);
        }
    }

    // Optionally shrink buffer to limit memory usage
    if (buffer.size() > 4096)
    {
        buffer.clear();
    }

    return frames;
}