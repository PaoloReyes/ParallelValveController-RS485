#include "../uart.h"

void uart_event_task(void* pvParameters){
    //Get the UART task data
    uart_task_data_t uart_task_data = *((uart_task_data_t*)pvParameters);
    //Create a uart data variables
    uart_event_t* event = new uart_event_t;
    size_t buffered_size;
    uint8_t* rx_data = (uint8_t*)malloc(UART_BUFFER_SIZE);
    //Placeholder for the pattern to be detected
    uint8_t* pattern = (uint8_t*)malloc(1);
    //Create a state machine for the UART
    fsm_state_t fsm_state = IDLE;

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_task_data.uart_queue, (void*)event, (TickType_t)portMAX_DELAY)) {
            //Clean the buffer
            bzero(rx_data, UART_BUFFER_SIZE);
            //Check the event type
            switch (event->type) {
                case UART_PATTERN_DET:
                    {
                        //Get the uart buffered size
                        uart_get_buffered_data_len(UART_NUM, &buffered_size);
                        //Get the position of the pattern
                        int pos = uart_pattern_pop_pos(UART_NUM);
                        //Check if the pattern is found
                        if (pos != -1) {
                            //Read the data from the UART
                            uart_read_bytes(UART_NUM, rx_data, pos, 100 / portTICK_PERIOD_MS);
                            //Read the linebreak to flush next pattern
                            uart_read_bytes(UART_NUM, pattern, buffered_size-pos, 100 / portTICK_PERIOD_MS);
                            ESP_LOGD(UART_TAG, "Read data: %s", rx_data);
                            //Perform the command based on the state machine
                            switch (fsm_state) {
                                case IDLE:
                                    {
                                        //Get the 2 character command
                                        char* command = get_command_from_data(rx_data);
                                        //Check whether the command is OP or AD
                                        if (strcmp(command, "OP") == 0) {
                                            //If command is OP, get the next buffered characters as the address and convert it to an integer
                                            int32_t address_numeric = get_int32_parameter_from_command(rx_data);
                                            //Check whether the address is the same as the current device address, send response and get the next state
                                            send_response_and_get_next_state(address_numeric == uart_task_data.device_data.address,
                                                                            MOVE_ON_SUCCESS, 
                                                                            WAITING_FOR_COMMAND,
                                                                            fsm_state);
                                        }
                                        free(command);
                                        break;
                                    }
                                case PENDING_CONFIGURATION:
                                    {
                                        //Get the 2 character command
                                        char* command = get_command_from_data(rx_data);
                                        if (strcmp(command, "WP") == 0) {
                                            //Update the address in the NVS
                                            esp_err_t err = ESP_OK;
                                            err = nvs_open(DEVICE_DATA_NAMESPACE, NVS_READWRITE, &uart_task_data.device_data.nvs_handle);
                                            err = nvs_set_i32(uart_task_data.device_data.nvs_handle, DEVICE_ADDRESS, uart_task_data.device_data.new_address);
                                            err = nvs_commit(uart_task_data.device_data.nvs_handle);
                                            nvs_close(uart_task_data.device_data.nvs_handle);
                                            send_response_and_get_next_state(err == ESP_OK,
                                                                            MOVE_ALWAYS, 
                                                                            WAITING_FOR_COMMAND,
                                                                            fsm_state);
                                        }
                                        free(command);
                                        break;  
                                    }
                                case WAITING_FOR_COMMAND:
                                    {
                                        //Get the 2 character command
                                        char* command = get_command_from_data(rx_data);
                                        if (strcmp(command, "CL") == 0) {
                                            printf("OK\n");
                                            fsm_state = IDLE;
                                        } else if (strcmp(command, "CV") == 0) {
                                            if (strlen((char*)rx_data) > 2) {
                                                char* parametes_string = get_string_parameter_from_command(rx_data);
                                                char* valve_id = (char*)malloc(2);
                                                char* weight = (char*)malloc(5);
                                                *valve_id = '\0';
                                                *weight = '\0';
                                                bool start_flag = false;
                                                for (int i = 0; i < strlen(parametes_string); i++) {
                                                    if (parametes_string[i] != ' ') {
                                                        start_flag = true;
                                                    }
                                                    if (parametes_string[i] == ' ' && start_flag) {
                                                        parametes_string[i] = '\0';
                                                        sprintf(valve_id, "%s", parametes_string);
                                                        sprintf(weight, "%s", parametes_string+i+1);
                                                        break;
                                                    }
                                                }
                                                if (strlen(valve_id) > 0 && strlen(weight) > 0 && are_all_digits(valve_id) && are_all_digits(weight)) {
                                                    uint8_t valve_id_numeric = atoi(valve_id);
                                                    uint16_t weight_numeric = atoi(weight);
                                                    printf("Valve ID: %d\n", valve_id_numeric);
                                                    printf("Weight: %d\n", weight_numeric);
                                                } else {
                                                    printf("ERR\n");
                                                }
                                                free(parametes_string);
                                                free(valve_id);
                                                free(weight);
                                            } else {
                                                printf("ERR\n");
                                            }
                                        } else if (strcmp(command, "AD") == 0) {
                                            //If command is AD, check whether the is an empty AD or has an address
                                            if (strlen((char*)rx_data) == 2) {
                                                //If the address is empty, return the current address
                                                printf("A:%03ld\n", uart_task_data.device_data.address);
                                            } else {
                                                //If the address is not empty, get the next buffered characters as the address and convert it to an integer
                                                uart_task_data.device_data.new_address = get_int32_parameter_from_command(rx_data);
                                                send_response_and_get_next_state(uart_task_data.device_data.new_address >= 0 && 
                                                                                uart_task_data.device_data.new_address <= 255 &&
                                                                                are_all_digits(get_string_parameter_from_command(rx_data)),
                                                                                MOVE_ON_SUCCESS, 
                                                                                PENDING_CONFIGURATION,
                                                                                fsm_state);
                                            }
                                        }
                                        free(command);
                                        break;
                                    }
                                default:
                                    break;
                            }
                        } else {
                            //If no pattern is found, flush the input buffer
                            uart_flush_input(UART_NUM);
                        }
                        break;
                    }
                default:
                    //Under abnormal conditions, log the event type
                    ESP_LOGD(UART_TAG, "UART event type: %d", event->type);
                    break;
            }
        }
    }
    //Delete the event and free the memory
    free(rx_data);
    rx_data = NULL;
    vTaskDelete(NULL);
}

/// @brief Gets the parameters after a two character command as a string (NEEDS TO FREE)
/// @param rx_data RX buffered data from uart pattern detection
/// @return Pointer to the string parameter
char* get_string_parameter_from_command(uint8_t* rx_data) {
    char* parameter = (char*)malloc(12);
    sprintf(parameter, "%s", (char*)rx_data+2);
    return parameter;
}

/// @brief Gets the parameters after a two character command
/// @param rx_data RX buffered data from uart pattern detection
/// @return 32-bit integer of the address
int32_t get_int32_parameter_from_command(uint8_t* rx_data) {
    char* parameter = get_string_parameter_from_command(rx_data);
    int32_t parameter_numeric = atoi(parameter);
    free(parameter);
    return parameter_numeric;
}

/// @brief Gets the command from the RX buffered data (NEEDS TO FREE)
/// @param rx_data RX buffered data from uart pattern detection
/// @return Two letter command
char* get_command_from_data(uint8_t* rx_data) {
    char* command = (char*)malloc(3);
    sprintf(command, "%c%c", rx_data[0], rx_data[1]);
    return command;
}

/// @brief Send an OK or ERR response and get the next state of the state machine
/// @param success Condition to evaluate the response
/// @param move_on When to move on to the next state
/// @param new_fsm_state State to move on to
/// @param current_fsm_state Current state of the state machine by reference
void send_response_and_get_next_state(bool success, move_on_t move_on, fsm_state_t new_fsm_state, fsm_state_t& current_fsm_state) {
    if (success) {
        printf("OK\n");
        if (move_on == MOVE_ON_SUCCESS) {
            current_fsm_state = new_fsm_state;
        }
    } else {
        printf("ERR\n");
        if (move_on == MOVE_ON_FAILURE) {
            current_fsm_state = new_fsm_state;
        }
    }
    if (move_on == MOVE_ALWAYS) {
        current_fsm_state = new_fsm_state;
    }
}

/// @brief Checks whether a string is all digits and spaces
/// @param string String to check
/// @return True if all characters are digits or spaces
bool are_all_digits(char* string) {
    for (int i = 0; i < strlen(string); i++) {
        if (!(isdigit(string[i]) || string[i] == ' ')) {
            return false;
        }
    }
    return true;
}