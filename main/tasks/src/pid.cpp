#include "../pid.h"

void PID_task_0(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 0);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_1(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 1);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_2(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 2);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_3(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 3);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_4(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 4);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_5(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 5);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_6(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 6);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void PID_task_7(void* pvParameters){
    void* pid_task_parameters = malloc(sizeof(void*));
    memcpy(pid_task_parameters, pvParameters, sizeof(void*));
    for (;;) {
        wait_notification_and_compute(pid_task_parameters, 7);
    }
    free(pid_task_parameters);
    vTaskDelete(NULL);
}

void wait_notification_and_compute(void* pvParameters, uint8_t task_id){
    ledc_channel_t ledc_channel = *((ledc_channel_t*)pvParameters);

    uint32_t ulNotifiedValue;
    xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);

    uint16_t weight = ulNotifiedValue>>16;
    uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
    write_angle(ledc_channel, weight);
    ESP_LOGI(PID_TAG, "Task %d Notified weight: %d and weight_setpoint: %d", task_id, weight, weight_setpoint);
}

double constrain(double value, double min, double max){
    if (value < min){
        return min;
    }
    if (value > max){
        return max;
    }
    return value;
}

double map(double value, double in_min, double in_max, double out_min, double out_max){
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void write_angle(ledc_channel_t pwm_channel, double angle){
    double seconds = map(constrain(angle, 0, 90), 0, 90, 0.001, 0.002);
    double time_step = 1/(PWM_FREQ*pow(2, DUTY_RESOLUTION));
    double duty = seconds/time_step;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);
}