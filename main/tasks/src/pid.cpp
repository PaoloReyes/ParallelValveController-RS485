#include "../pid.h"

void PID_task_0(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 0 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_1(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 1 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_2(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 2 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_3(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 3 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_4(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 4 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_5(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 5 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_6(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 6 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}

void PID_task_7(void* pvParameters){
    uint32_t ulNotifiedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        uint16_t weight = ulNotifiedValue>>16;
        uint16_t weight_setpoint = ulNotifiedValue&0x0000FFFF;
        ESP_LOGI(PID_TAG, "Task 7 Notified weight: %d and weight_setpoint: %d", weight, weight_setpoint);
    }
    vTaskDelete(NULL);
}