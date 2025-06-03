#include "uart-hal.hpp"
#include <esp_log.h>
#include <driver/gpio.h>

namespace lidar
{

    namespace
    {
        constexpr const char *TAG = "UART_HAL";
        constexpr std::size_t UART_BUF_SIZE = 1024;
    }

    UART_HAL::~UART_HAL()
    {
        deinit();
    }

    esp_err_t UART_HAL::init(const LiDARConfig &cfg)
    {
        uartPort_ = cfg.uartPort;

        const uart_config_t uartCfg = {
            .baud_rate = cfg.baudRate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
#if ESP_IDF_VERSION_MAJOR >= 4 && defined(UART_SCLK_DEFAULT)
            .source_clk = UART_SCLK_DEFAULT,
#endif
        };

        ESP_LOGI(TAG, "Initializing UART port %d with baud rate %d", uartPort_, cfg.baudRate);

        esp_err_t err = uart_param_config(uartPort_, &uartCfg);
        if (err != ESP_OK)
            return err;

        err = uart_set_pin(uartPort_, cfg.txPin, cfg.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK)
            return err;

        err = uart_driver_install(uartPort_, UART_BUF_SIZE, 0, 0, nullptr, 0);
        return err;
    }

    esp_err_t UART_HAL::read(std::span<std::byte> buffer, std::size_t &bytesRead, uint32_t timeoutMs) const
    {
        int len = uart_read_bytes(uartPort_, reinterpret_cast<uint8_t *>(buffer.data()), buffer.size(), pdMS_TO_TICKS(timeoutMs));
        if (len < 0)
        {
            bytesRead = 0;
            return ESP_FAIL;
        }

        bytesRead = static_cast<std::size_t>(len);
        return ESP_OK;
    }

    void UART_HAL::deinit()
    {
        uart_driver_delete(uartPort_);
    }

} // namespace lidar
