#include <driver/rmt_types.h>
#include <driver/rmt_rx.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

static const char *TAG = "rmt_rx";
// 1) Pick a big symbol depth. 128 is the max on most ESP32 RMT channels.
static constexpr size_t SYMBOL_BUFFER_SIZE = 64;
// 2) Your symbol buffer, sized in symbols
static rmt_symbol_word_t raw_symbols[SYMBOL_BUFFER_SIZE];

// ISR callback: forwards the done event to our queue
static bool IRAM_ATTR on_rmt_recv_done(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_data)
{
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)user_data, edata, &woken);
    return (bool)(woken == pdTRUE);
}

// Struct to hold everything the task needs
struct RmtTaskParams
{
    rmt_channel_handle_t chan;
    rmt_receive_config_t recv_cfg;
    rmt_symbol_word_t *raw_symbols;
    size_t buffer_bytes;
    QueueHandle_t queue;
};

// This is our FreeRTOS task (no captures!)
// It pulls events from the queue, logs them, and re-arms the RMT
static void rmt_rx_task(void *arg)
{
    ESP_LOGI(TAG, "[*]====> ENTER THE TASK");
    auto *p = static_cast<RmtTaskParams *>(arg);
    rmt_rx_done_event_data_t evt;
    while (xQueueReceive(p->queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Got %d symbols", evt.num_symbols);
        for (int i = 0; i < evt.num_symbols; i++)
        {
            auto &s = evt.received_symbols[i];
            ESP_LOGI(TAG, "  sym %2d: L0=%d @%dus  L1=%d @%dus",
                     i, s.level0, s.duration0, s.level1, s.duration1);
        }
        // re-arm for next frame
        ESP_ERROR_CHECK(rmt_receive(
            p->chan,
            p->raw_symbols,
            p->buffer_bytes,
            &p->recv_cfg));
    }
}

extern "C" void app_main()
{
    // 1) Allocate & configure RX channel
    rmt_channel_handle_t rx_chan = nullptr;
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = GPIO_NUM_18,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1 * 1000 * 1000, // 1 MHz → 1 µs ticks
        .mem_block_symbols = SYMBOL_BUFFER_SIZE,
        .intr_priority = 2,
        .flags{
            .invert_in = false,
            .with_dma = false,
            .allow_pd = false,
        },
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

    // 2) Create queue & register ISR callback
    QueueHandle_t recv_queue = xQueueCreate(3, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = on_rmt_recv_done};
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, recv_queue));

    ESP_ERROR_CHECK(rmt_enable(rx_chan));
    // 3) Prepare a receive config + buffer
    // static rmt_symbol_word_t raw_symbols[64]; // match mem_block_symbols
    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns = 500,              // ignore <1 µs
        .signal_range_max_ns = 10 * 1000 * 1000, // end frame at 10 ms idle
        .flags{
            .en_partial_rx = 1,
        },
    };
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols,
                                sizeof(raw_symbols), &recv_cfg));

    // 4) Bundle params and start the task
    static RmtTaskParams params = {
        .chan = rx_chan,
        .recv_cfg = recv_cfg,
        .raw_symbols = raw_symbols,
        .buffer_bytes = sizeof(raw_symbols),
        .queue = recv_queue,
    };
    TaskHandle_t rmt_task_handle = nullptr;
    BaseType_t res = xTaskCreate(rmt_rx_task,
                                 "rmt_rx_task",
                                 4096,
                                 &params,
                                 tskIDLE_PRIORITY + 1,
                                 &rmt_task_handle);
    if (res != pdPASS)
    {
        ESP_LOGE(TAG, "xTaskCreate failed! err=%d", res);
    }
    else
    {
        ESP_LOGI(TAG, "rmt_rx_task created, handle=%p", (void *)rmt_task_handle);
    }
}
