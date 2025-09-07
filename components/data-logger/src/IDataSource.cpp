#include "IDataSource.hpp"
#include <esp_timer.h>
#include <sstream>
#include <iomanip>

namespace digitoys::datalogger
{
    // DataEntry convenience constructors
    DataEntry::DataEntry(const std::string &k, int32_t v, uint64_t ts)
        : key(k), type(DataType::INT32), timestamp_us(ts)
    {
        value = std::to_string(v);
    }

    DataEntry::DataEntry(const std::string &k, uint32_t v, uint64_t ts)
        : key(k), type(DataType::UINT32), timestamp_us(ts)
    {
        value = std::to_string(v);
    }

    DataEntry::DataEntry(const std::string &k, float v, uint64_t ts)
        : key(k), type(DataType::FLOAT), timestamp_us(ts)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << v;
        value = oss.str();
    }

    DataEntry::DataEntry(const std::string &k, double v, uint64_t ts)
        : key(k), type(DataType::DOUBLE), timestamp_us(ts)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(10) << v;
        value = oss.str();
    }

    DataEntry::DataEntry(const std::string &k, const std::string &v, uint64_t ts)
        : key(k), type(DataType::STRING), value(v), timestamp_us(ts)
    {
    }

    DataEntry::DataEntry(const std::string &k, bool v, uint64_t ts)
        : key(k), type(DataType::BOOLEAN), timestamp_us(ts)
    {
        value = v ? "true" : "false";
    }

    const char *dataTypeToString(DataType type)
    {
        switch (type)
        {
        case DataType::INT32:
            return "int32";
        case DataType::UINT32:
            return "uint32";
        case DataType::FLOAT:
            return "float";
        case DataType::DOUBLE:
            return "double";
        case DataType::STRING:
            return "string";
        case DataType::BOOLEAN:
            return "boolean";
        default:
            return "unknown";
        }
    }

    uint64_t getCurrentTimestamp()
    {
        return esp_timer_get_time();
    }

} // namespace digitoys::datalogger
