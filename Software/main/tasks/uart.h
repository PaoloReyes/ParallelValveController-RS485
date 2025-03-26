#ifndef UART_TASKS_H
    #define UART_TASKS_H

    #include <string.h>
    #include <ctype.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <driver/uart.h>
    #include <esp_log.h>
    #include <nvs_flash.h>
    #include "../constants.h"
    #include "../data_types.h"

    typedef enum {
        IDLE,
        WAITING_FOR_COMMAND,
        PENDING_CONFIGURATION,
    } fsm_state_t;

    typedef enum {
        MOVE_ON_SUCCESS,
        MOVE_ON_FAILURE,
        MOVE_ALWAYS,
        DO_NOT_MOVE,
    } move_on_t;

    void uart_event_task(void *pvParameters);
    char* get_string_parameter_from_command(uint8_t* rx_data);
    char* get_command_from_data(uint8_t* rx_data);
    bool are_all_digits(char* string);
    int32_t get_next_number_from_string(char* &rx_data);
#endif