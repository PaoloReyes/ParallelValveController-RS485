#ifndef DEVICE_DATA_H
    #define DEVICE_DATA_H

    #include <stdint.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/queue.h>
    #include <driver/mcpwm.h>
    #include <driver/ledc.h>
    #include <nvs.h>

    typedef struct {
        int32_t address;
        int32_t new_address = -1;
        nvs_handle_t nvs_handle;
    } device_data_t;

    typedef struct {
        device_data_t device_data;
        QueueHandle_t uart_queue;
        TaskHandle_t* pid_tasks;
    } uart_task_data_t;

    typedef struct {
        mcpwm_unit_t mcpwm_unit;
        mcpwm_io_signals_t mcpwm_io_signal;
        ledc_channel_t ledc_channel;
        bool is_ledc;
    } pid_task_data_t;
#endif