#define UART_NUM UART_NUM_0   //UART number
#define UART_BUFFER_SIZE 256  //Size of the UART buffer
#define UART_BAUD_RATE 115200 //Baud rate for the UART
#define UART_QUEUE_SIZE 10    //Size of the UART queue

#define UART_TAG "UART" //TAG for the UART tasks
#define PID_TAG "PID"   //TAG for the PID tasks
#define NVS_TAG "NVS"   //TAG for the NVS

#define DEVICE_DATA_NAMESPACE "device_data"  //Namespace for the device data in the NVS
#define DEVICE_ADDRESS        "address"      //Address of the key value pair in the NVS for the device address

#define PWM_FREQ 50                       //PWM frequency
#define DUTY_RESOLUTION LEDC_TIMER_14_BIT //PWM resolution
//Define the pins and channels for the valves using the LEDC peripheral
#define VALVE0_PIN GPIO_NUM_4
#define VALVE0_CHANNEL LEDC_CHANNEL_0
#define VALVE1_PIN GPIO_NUM_5
#define VALVE1_CHANNEL LEDC_CHANNEL_1
#define VALVE2_PIN GPIO_NUM_6
#define VALVE2_CHANNEL LEDC_CHANNEL_2
#define VALVE3_PIN GPIO_NUM_7
#define VALVE3_CHANNEL LEDC_CHANNEL_3
#define VALVE4_PIN GPIO_NUM_15
#define VALVE4_CHANNEL LEDC_CHANNEL_4
#define VALVE5_PIN GPIO_NUM_16
#define VALVE5_CHANNEL LEDC_CHANNEL_5
#define VALVE6_PIN GPIO_NUM_17
#define VALVE6_CHANNEL LEDC_CHANNEL_6
#define VALVE7_PIN GPIO_NUM_18
#define VALVE7_CHANNEL LEDC_CHANNEL_7
//Define the pins for the valves using the MCPWM peripheral
#define VALVE8_PIN GPIO_NUM_8
#define VALVE9_PIN GPIO_NUM_3
#define VALVE10_PIN GPIO_NUM_46
#define VALVE11_PIN GPIO_NUM_9
#define VALVE12_PIN GPIO_NUM_10
#define VALVE13_PIN GPIO_NUM_11
#define VALVE14_PIN GPIO_NUM_12
#define VALVE15_PIN GPIO_NUM_13
#define VALVE16_PIN GPIO_NUM_14
#define VALVE17_PIN GPIO_NUM_21
#define VALVE18_PIN GPIO_NUM_47
#define VALVE19_PIN GPIO_NUM_48

#define VALVE0_KP 1.0
#define VALVE0_KI 0.0
#define VALVE0_KD 0.0

#define VALVE1_KP 1.0
#define VALVE1_KI 0.0
#define VALVE1_KD 0.0

#define VALVE2_KP 1.0
#define VALVE2_KI 0.0
#define VALVE2_KD 0.0

#define VALVE3_KP 1.0
#define VALVE3_KI 0.0
#define VALVE3_KD 0.0

#define VALVE4_KP 1.0
#define VALVE4_KI 0.0
#define VALVE4_KD 0.0

#define VALVE5_KP 1.0
#define VALVE5_KI 0.0
#define VALVE5_KD 0.0

#define VALVE6_KP 1.0
#define VALVE6_KI 0.0
#define VALVE6_KD 0.0

#define VALVE7_KP 1.0
#define VALVE7_KI 0.0
#define VALVE7_KD 0.0

#define VALVE8_KP 1.0
#define VALVE8_KI 0.0
#define VALVE8_KD 0.0

#define VALVE9_KP 1.0
#define VALVE9_KI 0.0
#define VALVE9_KD 0.0

#define VALVE10_KP 1.0
#define VALVE10_KI 0.0
#define VALVE10_KD 0.0

#define VALVE11_KP 1.0
#define VALVE11_KI 0.0
#define VALVE11_KD 0.0

#define VALVE12_KP 1.0
#define VALVE12_KI 0.0
#define VALVE12_KD 0.0

#define VALVE13_KP 1.0
#define VALVE13_KI 0.0
#define VALVE13_KD 0.0

#define VALVE14_KP 1.0
#define VALVE14_KI 0.0
#define VALVE14_KD 0.0

#define VALVE15_KP 1.0
#define VALVE15_KI 0.0
#define VALVE15_KD 0.0

#define VALVE16_KP 1.0
#define VALVE16_KI 0.0
#define VALVE16_KD 0.0

#define VALVE17_KP 1.0
#define VALVE17_KI 0.0
#define VALVE17_KD 0.0

#define VALVE18_KP 1.0
#define VALVE18_KI 0.0
#define VALVE18_KD 0.0

#define VALVE19_KP 1.0
#define VALVE19_KI 0.0
#define VALVE19_KD 0.0
