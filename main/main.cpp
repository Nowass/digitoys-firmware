#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "RMT_PWM";

// ==== Pin definitions ====
constexpr gpio_num_t PWM_GEN_GPIO = GPIO_NUM_6;   // Output (TX simulation)
constexpr gpio_num_t PWM_INPUT_GPIO = GPIO_NUM_7; // Input (RMT RX)
constexpr gpio_num_t PWM_OUT_GPIO = GPIO_NUM_8;   // Output (replayed pulse via GPIO)

// ==== Constants ====
constexpr uint32_t PWM_FREQ_HZ = 50;
constexpr uint32_t RMT_CLK_HZ = 1'000'000;     // 1 us ticks
constexpr uint32_t RMT_PWM_PERIOD_US = 20'000; // 20 ms

// ==== RMT handles ====
rmt_channel_handle_t rmt_rx_channel = nullptr;
rmt_channel_handle_t rmt_tx_channel = nullptr;
rmt_encoder_handle_t copy_encoder = nullptr;

// ==== Transmit configuration ====
static const rmt_transmit_config_t pwm_tx_config = {
    .loop_count = 0,          // single-shot
    .flags = {.eot_level = 0} // output low when done
};

// ==== Initialize RMT-based PWM generator ====
void init_rmt_pwm_generator()
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = PWM_GEN_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_CLK_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 8,
        .flags = {.invert_out = false}};
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &rmt_tx_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        abort();
    }
    err = rmt_enable(rmt_tx_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_enable tx channel failed: %s", esp_err_to_name(err));
        abort();
    }
    rmt_copy_encoder_config_t enc_cfg = {};
    err = rmt_new_copy_encoder(&enc_cfg, &copy_encoder);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %s", esp_err_to_name(err));
        abort();
    }
}

// ==== PWM generator task ====
void rmt_pwm_generator_task(void *param)
{
    // Pre-calc durations: 20% duty at 50Hz => 4ms high, 16ms low
    const uint32_t high_us = (RMT_PWM_PERIOD_US * 20) / 100;
    const uint32_t low_us = RMT_PWM_PERIOD_US - high_us;
    const uint32_t period_ms = RMT_PWM_PERIOD_US / 1000;
    rmt_symbol_word_t symbol = {};
    symbol.level0 = 1;
    symbol.duration0 = high_us;
    symbol.level1 = 0;
    symbol.duration1 = low_us;

    while (true)
    {
        ESP_LOGD(TAG, "TX pulse: %u us high, %u us low", high_us, low_us);
        // reset encoder state
        rmt_encoder_reset(copy_encoder);

        // transmit
        esp_err_t tx_err = rmt_transmit(rmt_tx_channel, copy_encoder,
                                        &symbol, sizeof(symbol), &pwm_tx_config);
        if (tx_err != ESP_OK)
        {
            ESP_LOGE(TAG, "rmt_transmit error: %s", esp_err_to_name(tx_err));
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        esp_err_t wait_err = rmt_tx_wait_all_done(rmt_tx_channel,
                                                  pdMS_TO_TICKS(period_ms + 1));
        if (wait_err != ESP_OK)
        {
            ESP_LOGE(TAG, "rmt_tx_wait_all_done error: %s", esp_err_to_name(wait_err));
        }
        // enforce period
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

// ==== Initialize RMT receiver ====
void init_rmt_rx()
{
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = PWM_INPUT_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_CLK_HZ,
        .mem_block_symbols = 64,
        .intr_priority = 0,
        .flags = {.invert_in = false, .with_dma = false, .allow_pd = false}};
    esp_err_t err = rmt_new_rx_channel(&rx_cfg, &rmt_rx_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s", esp_err_to_name(err));
        abort();
    }
    err = rmt_enable(rmt_rx_channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_enable rx channel failed: %s", esp_err_to_name(err));
        abort();
    }
}

// ==== Simple GPIO-based pulse output ====
void init_gpio_output()
{
    gpio_reset_pin(PWM_OUT_GPIO);
    gpio_set_direction(PWM_OUT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PWM_OUT_GPIO, 0);
}

// ==== PWM passthrough task ====
void pwm_passthrough_task(void *param)
{
    rmt_symbol_word_t symbols[8];
    rmt_receive_config_t cfg = {
        .signal_range_min_ns = 800000,
        .signal_range_max_ns = 2200000,
        .flags = {}};
    while (true)
    {
        esp_err_t err = rmt_receive(rmt_rx_channel, symbols,
                                    sizeof(symbols), &cfg);
        if (err == ESP_OK)
        {
            uint32_t h = symbols[0].duration0;
            ESP_LOGI(TAG, "RX pulse high: %u us", h);
            gpio_set_level(PWM_OUT_GPIO, 1);
            esp_rom_delay_us(h);
            gpio_set_level(PWM_OUT_GPIO, 0);
            esp_rom_delay_us(symbols[0].duration1);
        }
        else
        {
            ESP_LOGW(TAG, "rmt_receive error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    init_rmt_pwm_generator();
    init_rmt_rx();
    init_gpio_output();

    // Increase stack sizes to avoid overflow
    BaseType_t ret = xTaskCreate(rmt_pwm_generator_task,
                                 "rmt_pwm_gen", 8192, nullptr, 4, nullptr);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create rmt_pwm_generator_task");
        abort();
    }
    ret = xTaskCreate(pwm_passthrough_task,
                      "pwm_passthrough", 8192, nullptr, 5, nullptr);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create pwm_passthrough_task");
        abort();
    }
}
