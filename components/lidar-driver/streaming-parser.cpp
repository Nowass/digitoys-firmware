#include "streaming-parser.hpp"
#include <algorithm>
#include "esp_log.h"
#include "esp_system.h"

static constexpr const char *TAG = "StreamingParser";

std::vector<Frame> StreamingParser::parseStream(const uint8_t *data, size_t len)
{
    std::vector<Frame> frames;
    bool parsed = false;

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

        if (offset > 0)
        {
            ESP_LOGD(TAG, "↪ Resync: header found at offset %zu", offset);
        }

        // Try to parse frame from that location
        auto maybe_frame = frameParser.parseFrame(buffer.data() + offset, FrameParser::FRAME_LEN);

        if (maybe_frame.has_value())
        {
            frames.push_back(*maybe_frame);
            buffer.erase(buffer.begin(), buffer.begin() + offset + FrameParser::FRAME_LEN);
            parsed = true;
        }
        else
        {
            char hexbuf[100] = {};
            size_t dump_len = std::min<size_t>(16, buffer.size()); // dump up to 16 bytes
            for (size_t i = 0; i < dump_len; ++i)
            {
                sprintf(hexbuf + strlen(hexbuf), "%02X ", buffer[i]);
            }
            ESP_LOGW(TAG, "❌ Dropped invalid byte at offset %zu (resync), buffer: [%s]", offset, hexbuf);
            buffer.erase(buffer.begin(), buffer.begin() + offset + 1);
            parsed = true;
        }
    }

    // Suppress noise: only log if something seems really invalid
    if (!parsed && buffer.size() >= FrameParser::FRAME_LEN)
    {
        ESP_LOGW(TAG, "❌ No valid frame in %zu bytes, buffer size: %zu", len, buffer.size());
    }

    // Optionally shrink buffer to limit memory usage
    if (buffer.size() > 4096)
    {
        buffer.clear();
    }

    return frames;
}
