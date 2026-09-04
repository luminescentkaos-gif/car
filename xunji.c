#include <stdint.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "hi_io.h"
#include "ohos_init.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#define LEFT_SENSOR_GPIO WIFI_IOT_GPIO_IDX_13
#define RIGHT_SENSOR_GPIO WIFI_IOT_GPIO_IDX_14
#define SENSOR_ACTIVE_LEVEL WIFI_IOT_GPIO_VALUE1
#define UART_BAUD_RATE 115200
#define MOTOR_SPEED_MAX 150
#define BASE_SPEED 105
#define TURN_SPEED 200
#define SEARCH_SPEED 85
#define JUNCTION_SPEED 70
#define JUNCTION_DEFAULT_TURN 1
#define END_LINE_SPEED 45
#define END_CONFIRM_CYCLES 3
#define END_GAP_CONFIRM_CYCLES 3
#define END_MAX_GAP_CYCLES 120
#define LOOP_DELAY_MS 10
static uint8_t motor_frame[6];
static void motor_control(int left_speed, int right_speed)
{
    uint8_t left_direction = 0;
    uint8_t right_direction = 0;
    if (left_speed < 0) {
        left_direction = 1;
        left_speed = -left_speed;
    }
    if (right_speed < 0) {
        right_direction = 1;
        right_speed = -right_speed;
    }
    if (left_speed > MOTOR_SPEED_MAX) {
        left_speed = MOTOR_SPEED_MAX;
    }
    if (right_speed > MOTOR_SPEED_MAX) {
        right_speed = MOTOR_SPEED_MAX;
    }
    motor_frame[0] = 0xFC;
    motor_frame[1] = left_direction;
    motor_frame[2] = (uint8_t)left_speed;
    motor_frame[3] = right_direction;
    motor_frame[4] = (uint8_t)right_speed;
    motor_frame[5] = 0xFD;
    UartWrite(WIFI_IOT_UART_IDX_2, motor_frame, sizeof(motor_frame));
}
static void sensor_init(void)
{
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(LEFT_SENSOR_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(RIGHT_SENSOR_GPIO, WIFI_IOT_GPIO_DIR_IN);
}
static unsigned int sensor_level(uint32_t gpio)
{
    unsigned int value = WIFI_IOT_GPIO_VALUE1;
    GpioGetInputVal(gpio, &value);
    return value;
}
static void line_follow_task(void)
{
    uint8_t print_count = 0;
    uint8_t last_turn = JUNCTION_DEFAULT_TURN;
    uint8_t end_state = 0;
    uint8_t end_confirm_count = 0;
    uint8_t end_gap_count = 0;
    uint16_t end_gap_timer = 0;
    uint8_t stopped = 0;
    while (1) {
        unsigned int left_level = sensor_level(LEFT_SENSOR_GPIO);
        unsigned int right_level = sensor_level(RIGHT_SENSOR_GPIO);
        uint8_t left_on_line = left_level == SENSOR_ACTIVE_LEVEL;
        uint8_t right_on_line = right_level == SENSOR_ACTIVE_LEVEL;
        if (++print_count >= 20) {
            print_count = 0;
            printf("IR raw: L=%u R=%u, line: L=%u R=%u\n",
                left_level, right_level, left_on_line, right_on_line);
        }
        if (stopped) {
            motor_control(0, 0);
            osDelay(LOOP_DELAY_MS);
            continue;
        }
        /* 终点格式：稳定双黑 -> 离开双黑 -> 稳定双黑。 */
        if (end_state == 0) {
            if (left_on_line && right_on_line) {
                if (++end_confirm_count >= END_CONFIRM_CYCLES) {
                    end_state = 1;
                    end_confirm_count = 0;
                    motor_control(END_LINE_SPEED, END_LINE_SPEED);
                    printf("Finish marker 1 detected\n");
                }
            } else {
                end_confirm_count = 0;
            }
        } else if (end_state == 1) {
            motor_control(END_LINE_SPEED, END_LINE_SPEED);
            if (!(left_on_line && right_on_line)) {
                if (++end_gap_count >= END_GAP_CONFIRM_CYCLES) {
                    end_state = 2;
                    end_gap_count = 0;
                    end_gap_timer = 0;
                    printf("Finish gap detected\n");
                }
            } else {
                end_gap_count = 0;
            }
        } else {
            end_gap_timer++;
            if (left_on_line && right_on_line) {
                if (++end_confirm_count >= END_CONFIRM_CYCLES) {
                    motor_control(0, 0);
                    stopped = 1;
                    printf("Finish marker 2 detected, car stopped\n");
                }
            } else {
                end_confirm_count = 0;
            }
            if (end_gap_timer > END_MAX_GAP_CYCLES) {
                end_state = 0;
                end_gap_timer = 0;
                end_confirm_count = 0;
                printf("Finish sequence timeout, resume line follow\n");
            }
        }
        if (end_state != 0) {
            osDelay(LOOP_DELAY_MS);
            continue;
        }
        if (!left_on_line && !right_on_line) {
            motor_control(BASE_SPEED, BASE_SPEED);
        } else if (left_on_line && !right_on_line) {
            motor_control(-TURN_SPEED, TURN_SPEED);
            last_turn = 1;
        } else if (!left_on_line && right_on_line) {
            motor_control(TURN_SPEED, -TURN_SPEED);
            last_turn = 2;
        } else {
            /* Y 型岔路双黑时，延续进入岔路前的转向方向。 */
            if (last_turn == 1) {
                motor_control(-JUNCTION_SPEED, JUNCTION_SPEED);
            } else {
                motor_control(JUNCTION_SPEED, -JUNCTION_SPEED);
            }
        }
        osDelay(LOOP_DELAY_MS);
    }
}
static void line_follow_init(void)
{
    WifiIotUartAttribute uart_attribute = {
        .baudRate = UART_BAUD_RATE,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    osThreadAttr_t thread_attribute = {
        .name = "line_follow",
        .stack_size = 1024,
        .priority = 25,
    };
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart_attribute, NULL) != HI_ERR_SUCCESS) {
        printf("Failed to initialize motor UART\n");
        return;
    }
    sensor_init();
    if (osThreadNew((osThreadFunc_t)line_follow_task, NULL, &thread_attribute) == NULL) {
        printf("Failed to create line-follow task\n");
    }
}
APP_FEATURE_INIT(line_follow_init);
就在这个代码的基础上改,把原地转圈改成差速转圈,其他不要动