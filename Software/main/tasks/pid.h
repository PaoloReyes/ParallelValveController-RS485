#ifndef PID_H
    #define PID_H

    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <freertos/queue.h>
    #include <esp_log.h>
    #include <driver/ledc.h>
    #include <driver/mcpwm.h>
    #include <cmath>
    #include <cstring>
    #include <bits/stdc++.h>
    #include "../constants.h"
    #include "../data_types.h"

    void PID_task_0(void *pvParameters);
    void PID_task_1(void *pvParameters);
    void PID_task_2(void *pvParameters);
    void PID_task_3(void *pvParameters);
    void PID_task_4(void *pvParameters);
    void PID_task_5(void *pvParameters);
    void PID_task_6(void *pvParameters);
    void PID_task_7(void *pvParameters);
    void PID_task_8(void *pvParameters);
    void PID_task_9(void *pvParameters);
    void PID_task_10(void *pvParameters);
    void PID_task_11(void *pvParameters);
    void PID_task_12(void *pvParameters);
    void PID_task_13(void *pvParameters);
    void PID_task_14(void *pvParameters);
    void PID_task_15(void *pvParameters);
    void PID_task_16(void *pvParameters);
    void PID_task_17(void *pvParameters);
    void PID_task_18(void *pvParameters);
    void PID_task_19(void *pvParameters);

    //PID controller class
    class PID_controller{
        public:
            PID_controller(double kp, double ki, double kd);
            double compute(double setpoint, double input);
        private:
            double kp, ki, kd;
            double last_error, error_sum;
            double last_output;
            uint32_t last_time;
    };
    //PID tasks functions
    double angle_to_microseconds(double angle);
    void write_ledc_angle(ledc_channel_t ledc_channel, double angle);
    void write_mcpwm_angle(mcpwm_unit_t mcpwm_unit, mcpwm_io_signals_t mcpwm_io_signal, double angle);
    void wait_notification_and_compute(pid_task_data_t* pvParameters, PID_controller* pid_controller, uint8_t task_id);
    double constrain(double value, double min, double max);
    double map(double value, double in_min, double in_max, double out_min, double out_max);
#endif