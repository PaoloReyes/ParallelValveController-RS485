#ifndef PID_H
    #define PID_H

    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <freertos/queue.h>
    #include <esp_log.h>
    #include "../constants.h"


    void PID_task_0(void *pvParameters);
    void PID_task_1(void *pvParameters);
    void PID_task_2(void *pvParameters);
    void PID_task_3(void *pvParameters);
    void PID_task_4(void *pvParameters);
    void PID_task_5(void *pvParameters);
    void PID_task_6(void *pvParameters);
    void PID_task_7(void *pvParameters);
#endif