#ifndef DEVICE_DATA_H
    #define DEVICE_DATA_H

    #include <stdint.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/queue.h>

    typedef struct {
        int32_t address;
        int32_t new_address = -1;
        nvs_handle_t nvs_handle;
    } device_data_t;

    typedef struct {
        device_data_t device_data;
        QueueHandle_t uart_queue;
    } uart_task_data_t;
#endif