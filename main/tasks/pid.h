#ifndef PID_H
    #define PID_H

    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <freertos/queue.h>
    #include <esp_log.h>
    #include <driver/ledc.h>
    #include <cmath>
    #include <cstring>
    #include "../constants.h"

    void PID_task_0(void *pvParameters);
    void PID_task_1(void *pvParameters);
    void PID_task_2(void *pvParameters);
    void PID_task_3(void *pvParameters);
    void PID_task_4(void *pvParameters);
    void PID_task_5(void *pvParameters);
    void PID_task_6(void *pvParameters);
    void PID_task_7(void *pvParameters);

    void compute_pid(uint8_t task_number, uint16_t weight, uint16_t weight_setpoint);
    void write_angle(ledc_channel_t pwm_channel, double angle);
    void wait_notification_and_compute(void* pvParameters, uint8_t task_id);
    double constrain(double value, double min, double max);
    double map(double value, double in_min, double in_max, double out_min, double out_max);
#endif