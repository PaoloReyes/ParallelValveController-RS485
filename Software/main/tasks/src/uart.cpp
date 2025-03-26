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
                            //Get the 2 character command and the string parameters
                            char* command = get_command_from_data(rx_data);
                            char* parameters = get_string_parameter_from_command(rx_data);
                            char* start_address_parameters = parameters;
                            //Perform the command based on the state machine
                            switch (fsm_state) {
                                case IDLE:
                                    {
                                        //Check whether the command is OP or AD
                                        if (strcmp(command, "OP") == 0) {
                                            //If command is OP, get the next buffered characters as the address and convert it to an integer
                                            int32_t address_numeric = get_next_number_from_string(parameters);
                                            //Check whether the address is the same as the current device address, send response and get the next state
                                            if (address_numeric == uart_task_data.device_data.address) {
                                                fsm_state = WAITING_FOR_COMMAND;
                                                printf("OK\n");
                                            }
                                        }
                                        break;
                                    }
                                case PENDING_CONFIGURATION:
                                    {
                                        if (strcmp(command, "WP") == 0) {
                                            //Update the address in the NVS
                                            nvs_open(DEVICE_DATA_NAMESPACE, NVS_READWRITE, &uart_task_data.device_data.nvs_handle);
                                            nvs_set_i32(uart_task_data.device_data.nvs_handle, DEVICE_ADDRESS, uart_task_data.device_data.new_address);
                                            nvs_commit(uart_task_data.device_data.nvs_handle);
                                            nvs_close(uart_task_data.device_data.nvs_handle);
                                            fsm_state = WAITING_FOR_COMMAND;
                                            printf("OK\n");
                                        } else {
                                            printf("ERR\n");
                                        }
                                        break;  
                                    }
                                case WAITING_FOR_COMMAND:
                                    {
                                        if (strcmp(command, "CL") == 0) {
                                            printf("OK\n");
                                            fsm_state = IDLE;
                                        } else if (strcmp(command, "CV") == 0) {
                                            int32_t valve_id = get_next_number_from_string(parameters);
                                            int32_t weight = get_next_number_from_string(parameters);
                                            int32_t weight_setpoint = get_next_number_from_string(parameters);
                                            if (valve_id >= 0 && valve_id <= 19 && weight >= 0 && weight <= 65535 &&
                                                weight_setpoint >= 0 && weight_setpoint <= 65535) {
                                                printf("OK\n");
                                                uint32_t value_to_send = ((uint16_t)weight << 16) | (uint16_t)weight_setpoint;
                                                xTaskNotify(uart_task_data.pid_tasks[valve_id], value_to_send, eSetValueWithOverwrite);
                                            } else {
                                                printf("ERR\n");
                                            }
                                        } else if (strcmp(command, "AD") == 0) {
                                            //If command is AD, check whether the is an empty AD or has an address
                                            int32_t parameter_number = get_next_number_from_string(parameters);
                                            if (parameter_number == -1) {
                                                //If the address is empty, return the current address
                                                printf("A:%03ld\n", uart_task_data.device_data.address);
                                            } else {
                                                //If the address is not empty, get the next buffered characters as the address and convert it to an integer
                                                uart_task_data.device_data.new_address = parameter_number;
                                                if (uart_task_data.device_data.new_address >= 0 && uart_task_data.device_data.new_address <= 255) {
                                                    fsm_state = PENDING_CONFIGURATION;
                                                    printf("OK\n");
                                                } else {
                                                    printf("ERR\n");
                                                }	
                                            }
                                        } else {
                                            printf("ERR\n");
                                        }
                                        break;
                                    }
                                default:
                                    break;
                            }
                            //Free the memory
                            free(command);
                            free(start_address_parameters);
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
    char* parameter = (char*)malloc(20);
    sprintf(parameter, "%s", (char*)rx_data+2);
    return parameter;
}

/// @brief Extracts the next number from a string and updates the string pointer
/// @param string Pointer to the string to extract the number from
/// @return First number in the string
int32_t get_next_number_from_string(char* &string) {
    int32_t parameter_numeric;
    uint16_t i;
    bool start_search = false;
    for (i = 0; i < strlen(string); i++) {
        if (string[i] != ' ') {
            start_search = true;
        }
        if (string[i] == ' ' && start_search) {
            string[i] = '\0';
            break;
        }
    }
    are_all_digits(string) ? parameter_numeric = atoi(string) : parameter_numeric = -1;
    string+=i+1;
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

/// @brief Checks whether a string is all digits and spaces
/// @param string String to check
/// @return True if all characters are digits or spaces
bool are_all_digits(char* string) {
    uint8_t digits_count = 0;
    uint8_t spaces_cr_count = 0;
    for (int i = 0; i < strlen(string); i++) {
        if (isdigit(string[i])) {
            digits_count++;
        } else if ((string[i] == ' ') || (string[i] == '\r')) {
            spaces_cr_count++;
        } else {
            return false;
        }
    }

    return (digits_count != 0);
}