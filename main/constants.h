#define UART_NUM UART_NUM_0   //UART number
#define UART_BUFFER_SIZE 256  //Size of the UART buffer
#define UART_BAUD_RATE 115200 //Baud rate for the UART
#define UART_QUEUE_SIZE 10    //Size of the UART queue

#define UART_TAG "UART" //TAG for the UART tasks
#define PID_TAG "PID"   //TAG for the PID tasks
#define NVS_TAG "NVS"   //TAG for the NVS

#define DEVICE_DATA_NAMESPACE "device_data"  //Namespace for the device data in the NVS
#define DEVICE_ADDRESS        "address"      //Address of the key value pair in the NVS for the device address

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 //1MHz
#define SERVO_TIMEBASE_PERIOD        20000   //20000 ticks = 20ms

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