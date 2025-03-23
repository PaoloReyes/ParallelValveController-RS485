#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/ledc.h>
#include <driver/mcpwm.h>
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
    //Read Address and set to DEVICE_ID if not found or different
    if (nvs_get_i32(device_data.nvs_handle, DEVICE_ADDRESS, &device_data.address) != ESP_OK) {
        nvs_set_i32(device_data.nvs_handle, DEVICE_ADDRESS, 0);
        nvs_commit(device_data.nvs_handle);
    }
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
    //Create an array for ledc channels
    ledc_channel_config_t ledc_channels[8];
    //ledc channel0 configuration and defaults angle to 0
    ledc_channels[0] = {
        .gpio_num = VALVE0_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = VALVE0_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channels[0]));
    write_ledc_angle(ledc_channels[0].channel, 0);

    //Create an array for pins and channels
    gpio_num_t valve_pins_ledc[7] = {VALVE1_PIN, VALVE2_PIN, VALVE3_PIN, VALVE4_PIN, VALVE5_PIN, VALVE6_PIN, VALVE7_PIN};
    ledc_channel_t valve_channels_ledc[7] = {VALVE1_CHANNEL, VALVE2_CHANNEL, VALVE3_CHANNEL, VALVE4_CHANNEL, VALVE5_CHANNEL, VALVE6_CHANNEL, VALVE7_CHANNEL};
    //Copy the ledc_channel0 configuration to other ledc_channels configuration, update pin and channel, apply configurations and default angles
    for (int i = 1; i < 8; i++){
        memcpy(&ledc_channels[i], &ledc_channels[0], sizeof(ledc_channel_config_t));
        ledc_channels[i].gpio_num = valve_pins_ledc[i-1];
        ledc_channels[i].channel = valve_channels_ledc[i-1];
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channels[i]));
        write_ledc_angle(ledc_channels[i].channel, 0);
    }

    //Create an array for mcwpm pins
    gpio_num_t valve_pins_mcpwm0[6] = {VALVE8_PIN, VALVE9_PIN, VALVE10_PIN, VALVE11_PIN, VALVE12_PIN, VALVE13_PIN};
    gpio_num_t valve_pins_mcpwm1[6] = {VALVE14_PIN, VALVE15_PIN, VALVE16_PIN, VALVE17_PIN, VALVE18_PIN, VALVE19_PIN};
    //Initializa the mcpwm modules
    for (int i = 0; i < 6; i++){
        ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, (mcpwm_io_signals_t)i, valve_pins_mcpwm0[i]));
        ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_1, (mcpwm_io_signals_t)i, valve_pins_mcpwm1[i]));
    }
    //Create a configuration for the mcpwm timers
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    //Initialize the mcpwm timers, each timer will control two operators
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &pwm_config));
    //Set the initial angle to 0 for all the mcpwm operators
    for (int i = 0; i < 6; i++){
        write_mcpwm_angle(MCPWM_UNIT_0, (mcpwm_io_signals_t)i, 0);
        write_mcpwm_angle(MCPWM_UNIT_1, (mcpwm_io_signals_t)i, 0);
    }

    //Create an array of PID task functions to create the tasks in a loop
    TaskFunction_t PID_tasks[20] = {PID_task_0, PID_task_1, PID_task_2, PID_task_3, PID_task_4, 
        PID_task_5, PID_task_6, PID_task_7, PID_task_8, PID_task_9, 
        PID_task_10, PID_task_11, PID_task_12, PID_task_13, PID_task_14, 
        PID_task_15, PID_task_16, PID_task_17, PID_task_18, PID_task_19};
    //Create an array of names for the PID tasks to create the tasks in a loop
    const char* PID_tasks_names[20] = {"PID_task_0", "PID_task_1", "PID_task_2", "PID_task_3", "PID_task_4", 
            "PID_task_5", "PID_task_6", "PID_task_7", "PID_task_8", "PID_task_9", 
            "PID_task_10", "PID_task_11", "PID_task_12", "PID_task_13", "PID_task_14", 
            "PID_task_15", "PID_task_16", "PID_task_17", "PID_task_18", "PID_task_19"};
    //Create an array of pid_task_data_t for the PID tasks
    pid_task_data_t pid_task_data[20];
    //Fill ledc data
    for (int i = 0; i < 8; i++){
        pid_task_data[i].ledc_channel = ledc_channels[i].channel;
        pid_task_data[i].is_ledc = true;
    }
    //Fill mcpwm data
    for (int i = 8; i < 14; i++){
        pid_task_data[i].mcpwm_unit = MCPWM_UNIT_0;
        pid_task_data[i].mcpwm_io_signal = (mcpwm_io_signals_t)(i-8);
        pid_task_data[i].is_ledc = false;
    }
    for (int i = 14; i < 20; i++){
        pid_task_data[i].mcpwm_unit = MCPWM_UNIT_1;
        pid_task_data[i].mcpwm_io_signal = (mcpwm_io_signals_t)(i-14);
        pid_task_data[i].is_ledc = false;
    }
    //Create a task handle array for PID tasks
    TaskHandle_t* pid_tasks = (TaskHandle_t*)malloc(sizeof(TaskHandle_t)*20); 
    //Create individual tasks for PID control for multicore processing
    for (int i = 0; i < 20; i++){
        xTaskCreatePinnedToCore(PID_tasks[i], PID_tasks_names[i], 4096, &pid_task_data[i], 1, &pid_tasks[i], tskNO_AFFINITY);
    }

    //Create an uart task data structure
    uart_task_data_t uart_task_data = {
        .device_data = device_data,
        .uart_queue = uart_queue,
        .pid_tasks = pid_tasks,
    };

    //Create a task to handle UART event from ISR
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 8192, (void*)&uart_task_data, 2, NULL, tskNO_AFFINITY);
}