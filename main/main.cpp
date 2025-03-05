#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/ledc.h>
#include "tasks/uart.h"
#include "tasks/pid.h"
#include "constants.h"
#include "data_types.h"

extern "C" void app_main(void) {
    // Set the log level for the UART task
    esp_log_level_set(UART_TAG, ESP_LOG_INFO);

    // Create a queue to handle UART event from ISR
    QueueHandle_t uart_queue;
    // UART Initialization
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Set UART configuration parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, UART_QUEUE_SIZE, &uart_queue, 0));
    //Set uart pattern detect function.
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_NUM, '\n', 1, 9, 0, 0));
    //Reset the pattern queue length to record at most UART_QUEUE_SIZE pattern positions.
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_NUM, UART_QUEUE_SIZE));

    //NVS Flash initialization
    ESP_ERROR_CHECK(nvs_flash_init());
    //Create a device data structure and nvs handle
    device_data_t device_data;
    //Open NVS
    ESP_ERROR_CHECK(nvs_open(DEVICE_DATA_NAMESPACE, NVS_READWRITE, &device_data.nvs_handle));
    //Read Address
    ESP_ERROR_CHECK(nvs_get_i32(device_data.nvs_handle, DEVICE_ADDRESS, &device_data.address));
    //Close
    nvs_close(device_data.nvs_handle);

    //ledc timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    //ledc channel0 configuration and defaults angle to 0
    ledc_channel_config_t ledc_channel_0 = {
        .gpio_num = VALVE0_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = VALVE0_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));
    write_angle(ledc_channel_0.channel, 0);

    //Create an array for pins and channels
    gpio_num_t valve_pins[7] = {VALVE1_PIN, VALVE2_PIN, VALVE3_PIN, VALVE4_PIN, VALVE5_PIN, VALVE6_PIN, VALVE7_PIN};
    ledc_channel_t valve_channels[7] = {VALVE1_CHANNEL, VALVE2_CHANNEL, VALVE3_CHANNEL, VALVE4_CHANNEL, VALVE5_CHANNEL, VALVE6_CHANNEL, VALVE7_CHANNEL};
    //Copy the ledc_channel_0 configuration to other ledc_channel configurations, update pin and channel, apply configurations and default angles
    ledc_channel_config_t ledc_rest_channels[7];
    for (int i = 0; i < 7; i++){
        memcpy(&ledc_rest_channels[i], &ledc_channel_0, sizeof(ledc_channel_config_t));
        ledc_rest_channels[i].gpio_num = valve_pins[i];
        ledc_rest_channels[i].channel = valve_channels[i];
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_rest_channels[i]));
        write_angle(ledc_rest_channels[i].channel, 0);
    }

    //Create a task handle array for PID tasks
    TaskHandle_t* pid_tasks = (TaskHandle_t*)malloc(sizeof(TaskHandle_t)*8); 
    //Create individual tasks for PID control for multicore processing
    xTaskCreatePinnedToCore(PID_task_0, "PID_task_0", 4096, &ledc_channel_0.channel, 1, &pid_tasks[0], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_1, "PID_task_1", 4096, &ledc_rest_channels[0].channel, 1, &pid_tasks[1], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_2, "PID_task_2", 4096, &ledc_rest_channels[1].channel, 1, &pid_tasks[2], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_3, "PID_task_3", 4096, &ledc_rest_channels[2].channel, 1, &pid_tasks[3], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_4, "PID_task_4", 4096, &ledc_rest_channels[3].channel, 1, &pid_tasks[4], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_5, "PID_task_5", 4096, &ledc_rest_channels[4].channel, 1, &pid_tasks[5], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_6, "PID_task_6", 4096, &ledc_rest_channels[5].channel, 1, &pid_tasks[6], tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PID_task_7, "PID_task_7", 4096, &ledc_rest_channels[6].channel, 1, &pid_tasks[7], tskNO_AFFINITY);

    //Create an uart task data structure
    uart_task_data_t uart_task_data = {
        .device_data = device_data,
        .uart_queue = uart_queue,
        .pid_tasks = pid_tasks,
    };

    //Create a task to handle UART event from ISR
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 8192, (void*)&uart_task_data, 2, NULL, tskNO_AFFINITY);
}