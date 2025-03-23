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
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE0_KP, VALVE0_KI, VALVE0_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 0);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve1
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_1(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE1_KP, VALVE1_KI, VALVE1_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 1);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve2
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_2(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE2_KP, VALVE2_KI, VALVE2_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 2);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve3
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_3(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE3_KP, VALVE3_KI, VALVE3_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 3);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve4
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_4(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE4_KP, VALVE4_KI, VALVE4_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 4);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve5
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_5(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE5_KP, VALVE5_KI, VALVE5_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 5);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve6
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_6(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE6_KP, VALVE6_KI, VALVE6_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 6);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve7
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_7(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE7_KP, VALVE7_KI, VALVE7_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 7);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve8
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_8(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE8_KP, VALVE8_KI, VALVE8_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 8);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve9
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_9(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE9_KP, VALVE9_KI, VALVE9_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 9);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve10
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_10(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE10_KP, VALVE10_KI, VALVE10_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 10);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve11
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_11(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE11_KP, VALVE11_KI, VALVE11_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 11);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve12
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_12(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE12_KP, VALVE12_KI, VALVE12_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 12);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve13
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_13(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE13_KP, VALVE13_KI, VALVE13_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 13);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve14
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_14(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE14_KP, VALVE14_KI, VALVE14_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 14);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve15
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_15(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE15_KP, VALVE15_KI, VALVE15_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 15);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve16
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_16(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE16_KP, VALVE16_KI, VALVE16_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 16);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve17
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_17(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE17_KP, VALVE17_KI, VALVE17_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 17);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve18
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_18(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE18_KP, VALVE18_KI, VALVE18_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 18);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Perform the control on valve19
/// @param pvParameters Void poiner to usefull data for the task
void PID_task_19(void* pvParameters){
    pid_task_data_t pid_task_data = *((pid_task_data_t*)pvParameters);
    PID_controller pid_controller(VALVE19_KP, VALVE19_KI, VALVE19_KD);
    for (;;) {
        wait_notification_and_compute(&pid_task_data, &pid_controller, 19);
    }
    free(&pid_task_data);
    vTaskDelete(NULL);
}

/// @brief Function to generalize computation on the tasks
/// @param pvParameters Void pointer to the paramaters of the task
/// @param task_id Task id to identify the task
void wait_notification_and_compute(pid_task_data_t* pvParameters, PID_controller* pid_controller, uint8_t task_id) {
    pid_task_data_t pid_task_data = *pvParameters;

    uint32_t ulNotifiedValue;
    xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
    uint16_t weight = ulNotifiedValue>>16;
    uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;

    double angle = pid_controller->compute(weight_setpoint, weight);
    if (pid_task_data.is_ledc) {
        ledc_channel_t ledc_channel = pid_task_data.ledc_channel;
        write_ledc_angle(ledc_channel, angle);
    } else {
        mcpwm_unit_t mcpwm_unit = pid_task_data.mcpwm_unit;
        mcpwm_io_signals_t mcpwm_io_signal = pid_task_data.mcpwm_io_signal;
        write_mcpwm_angle(mcpwm_unit, mcpwm_io_signal, angle);
    }
    ESP_LOGI(PID_TAG, "Task %d Notified weight: %d and weight_setpoint: %d and pid: %f", task_id, weight, weight_setpoint, angle);
}

PID_controller::PID_controller(double kp, double ki, double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->last_error = 0;
    this->error_sum = 0;
    this->last_time = 0;
}

double PID_controller::compute(double setpoint, double input){
    uint32_t time = xTaskGetTickCount()*(1000/configTICK_RATE_HZ);
    double error = setpoint - input;
    if (time > this->last_time){
        double delta_time = (time - this->last_time)/1000.0;
        double p = kp*error;
        if (abs(error) < 10){
            this->error_sum += error*delta_time;
        } else {
            this->error_sum = 0;
        }
        double i = ki*this->error_sum;
        double d = kd*(error - this->last_error)/delta_time;
        this->last_output = p + i + d;
    }
    this->last_error = error;
    this->last_time = time;
    return this->last_output;
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

/// @brief Convert an angle to microseconds
/// @param angle Angle to convert
/// @return Angle in microseconds
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
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel));
}

/// @brief Write the angle to the servo motor using mcpwm
/// @param mcpwm_unit mcpwm unit to write the angle
/// @param mcpwm_io_signal mcpwm io signal to write the angle
/// @param angle Angle to write to the servo motor
void write_mcpwm_angle(mcpwm_unit_t mcpwm_unit, mcpwm_io_signals_t mcpwm_io_signal, double angle){
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(mcpwm_unit, mcpwm_io2timer[mcpwm_io_signal], mcpwm_io2generator[mcpwm_io_signal], map(constrain(angle, 0, 90), 0, 90, 1000, 2000)));
}