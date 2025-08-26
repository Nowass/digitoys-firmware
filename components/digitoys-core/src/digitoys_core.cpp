#include "digitoys_core.hpp"
#include <sstream>

namespace digitoys
{

    esp_err_t initializeFramework()
    {
        // Initialize logging subsystem
        esp_err_t result = logging::utils::initializeLogging();
        if (result != ESP_OK)
        {
            return result;
        }

        // Create a framework logger
        logging::ComponentLogger logger("DIGITOYS-CORE");

        // Log framework initialization
        logger.info("DigiToys Core Framework v%s initializing...", framework::VERSION);
        logger.info("Build: %s %s", framework::BUILD_DATE, framework::BUILD_TIME);

        // Log system information
        logger.logMemoryInfo();
        logger.logTaskInfo();

        logger.info("DigiToys Core Framework initialized successfully");

        return ESP_OK;
    }

    std::string getFrameworkVersion()
    {
        return std::string(framework::VERSION);
    }

    std::string getFrameworkBuildInfo()
    {
        std::ostringstream oss;
        oss << "DigiToys Core Framework v" << framework::VERSION;
        oss << " (Built: " << framework::BUILD_DATE << " " << framework::BUILD_TIME << ")";
        return oss.str();
    }

} // namespace digitoys
