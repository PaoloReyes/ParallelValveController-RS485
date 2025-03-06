#include "../pid.h"

static std::map<mcpwm_io_signals_t, mcpwm_generator_t> mcpwm_io2generator = {
    {MCPWM0A, MCPWM_GEN_A},
    {MCPWM0B, MCPWM_GEN_B},
    {MCPWM1A, MCPWM_GEN_A},
    {MCPWM1B, MCPWM_GEN_B},
    {MCPWM2A, MCPWM_GEN_A},
    {MCPWM2B, MCPWM_GEN_B},
};

static std::map<mcpwm_io_signals_t, mcpwm_timer_t> mcpwm_io2timer = {
    {MCPWM0A, MCPWM_TIMER_0},
    {MCPWM0B, MCPWM_TIMER_0},
    {MCPWM1A, MCPWM_TIMER_1},
    {MCPWM1B, MCPWM_TIMER_1},
    {MCPWM2A, MCPWM_TIMER_2},
    {MCPWM2B, MCPWM_TIMER_2},
};

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_0(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 0);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_1(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 1);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_2(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 2);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_3(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 3);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_4(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 4);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_5(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 5);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_6(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 6);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve0
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_7(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 7);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

/// @brief Function to generalize computation on the tasks
/// @param pvParameters Void pointer to the paramaters of the task
/// @param task_id Task id to identify the task
void wait_notification_and_compute(void* pvParameters, uint8_t task_id){
    ledc_channel_t ledc_channel = *((ledc_channel_t*)pvParameters);

    uint32_t ulNotifiedValue;
    xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);

    uint16_t weight = ulNotifiedValue>>16;
    uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
    write_ledc_angle(ledc_channel, weight);
    ESP_LOGI(PID_TAG, "Task %d Notified weight: %d and weight_setpoint: %d", task_id, weight, weight_setpoint);
}

/// @brief Limit a value to a range
/// @param value Value to be constrained
/// @param min Minimum value of the range
/// @param max Maximum value of the range
/// @return Constraint value
double constrain(double value, double min, double max){
    if (value < min){
        return min;
    }
    if (value > max){
        return max;
    }
    return value;
}

/// @brief Map a value from one range to another
/// @param value Value to be mapped
/// @param in_min Minimum value of the input range
/// @param in_max Maximum value of the input range
/// @param out_min Minimum value of the output range
/// @param out_max Minimum value of the output range
/// @return Mapped value
double map(double value, double in_min, double in_max, double out_min, double out_max){
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double angle_to_microseconds(double angle){
    return map(constrain(angle, 0, 90), 0, 90, 1000, 2000);
}

/// @brief Write the angle to the servo motor using ledc
/// @param ledc_channel ledc channel to write the angle
/// @param angle Angle to write to the servo motor
void write_ledc_angle(ledc_channel_t ledc_channel, double angle){
    double seconds = angle_to_microseconds(angle)/1000000;
    double time_step = 1/(PWM_FREQ*pow(2, DUTY_RESOLUTION));
    double duty = seconds/time_step;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel);
}

void write_mcpwm_angle(mcpwm_unit_t mcpwm_unit, mcpwm_io_signals_t mcpwm_io_signal, double angle){
    mcpwm_set_duty_in_us(mcpwm_unit, mcpwm_io2timer[mcpwm_io_signal], mcpwm_io2generator[mcpwm_io_signal], map(constrain(angle, 0, 90), 0, 90, 1000, 2000));
}