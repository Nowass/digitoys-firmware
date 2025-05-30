#pragma once

#include "frame-parser.hpp"
#include <vector>
#include <cstdint>

class StreamingParser
{
public:
    std::vector<Frame> parseStream(const uint8_t *data, size_t len);

private:
    std::vector<uint8_t> buffer;
    FrameParser frameParser;
};

// streaming_parser.cpp