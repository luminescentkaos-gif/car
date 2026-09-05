#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"
static void thread_8(void);
static void HCSR04_Init(void);
static float HCSR04_GetDistance_cm(void);
static void TrackSensor_Init(void);
static uint8_t is_edge_detected(void);
static void avoid_obstacle(int turn_left);
uint8_t uart_sendbuf[20];
osMutexId_t mutex_id;
/* 发送速度上限：电机实际转速的一百倍（0~255，超出会被截断，默认150=1.5rad/s） */
#define SPEED_MAX    150
/* ============================
 * 超声波避障 + 巡线传感器防掉桌
 *
 * 避障：HC-SR04 朝前，距离 < OBSTACLE_DIST_CM 判定有障碍
 * 防掉桌：板载双路红外巡线传感器（朝下），左路 IO13、右路 IO14，
 *        在桌面上反射强输出高电平(1)，到桌边反射弱输出低电平(0)，
 *        任意一路变低即判定为桌边悬空。
 *        不需要调超声波角度。
 * ============================ */
#define OBSTACLE_DIST_CM   20    /* 超声波小于此距离判定为有障碍 */
#define AVOID_SPEED        100   /* 避障动作速度（转速×100） */
#define BACKUP_MS          500   /* 后退持续时间(ms) */
#define TURN_MS            400   /* 转向持续时间(ms) */
#define TICK_MS            50    /* 测距时间片(ms)，越小反应越快 */
/***通信协议***/
/*
函数功能：发送至stm32的数据协议
参数 ：电机实际转速的一百倍，例如：设置转速为1rad/s，则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;
    //小车运动方向 前进（正转）：0 后退（反转） 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }
    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }
    //限制幅度 -SPEED_MAX ~ SPEED_MAX
    if (motorA > SPEED_MAX)
    {
        motorA = SPEED_MAX;
    }
    if (motorB > SPEED_MAX)
    {
        motorB = SPEED_MAX;
    }
    // 数据协议
    uart_sendbuf[0] = 0xFC; // 帧头
    uart_sendbuf[1] = A_dir; // 左轮方向 0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir; // 右轮方向 0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD; // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}
/* ============================
 * HC-SR04 超声波初始化
 * Trig -> GPIO_7  推挽输出
 * Echo -> GPIO_8  浮空输入
 * ============================ */
static void HCSR04_Init(void)
{
    // Trig: IO07 设为 GPIO 功能 + 输出
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_7, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(WIFI_IOT_GPIO_IDX_7, WIFI_IOT_GPIO_VALUE0);
    // Echo: IO08 设为 GPIO 功能 + 输入
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_8, WIFI_IOT_GPIO_DIR_IN);
}
/* ============================
 * 超声波测距，返回距离(cm)
 * 超时或无回波返回 -1.0
 * ============================ */
static float HCSR04_GetDistance_cm(void)
{
    unsigned int echo_val = 0;
    unsigned int pulse_us = 0;
    unsigned int timeout = 0;
    /* 发 20us 触发脉冲（手册要求至少 10us） */
    GpioSetOutputVal(WIFI_IOT_GPIO_IDX_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(WIFI_IOT_GPIO_IDX_7, WIFI_IOT_GPIO_VALUE0);
    /* 等待 Echo 上升沿（超时 50ms） */
    timeout = 0;
    do {
        GpioGetInputVal(WIFI_IOT_GPIO_IDX_8, &echo_val);
        if(++timeout > 50000) return -1.0f;
        hi_udelay(1);
    } while(echo_val == WIFI_IOT_GPIO_VALUE0);
    /* 计时 Echo 高电平宽度（最大 30ms，约 4m） */
    timeout = 0;
    do {
        GpioGetInputVal(WIFI_IOT_GPIO_IDX_8, &echo_val);
        pulse_us++;
        hi_udelay(1);
        if(++timeout > 30000) break;
    } while(echo_val == WIFI_IOT_GPIO_VALUE1);
    /* 距离(cm) = 时间(us) × 声速(0.034cm/us) ÷ 2 */
    return (float)pulse_us * 0.017f;
}
/* ============================
 * 巡线传感器初始化（防掉桌用）
 * 左路 -> GPIO_13  输入
 * 右路 -> GPIO_14  输入
 * 板载 TCRT5000 红外对管，朝下安装
 * ============================ */
static void TrackSensor_Init(void)
{
    // 左路巡线: IO13 设为 GPIO 功能 + 输入
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_13, WIFI_IOT_GPIO_DIR_IN);
    // 右路巡线: IO14 设为 GPIO 功能 + 输入
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_14, WIFI_IOT_GPIO_DIR_IN);
}
/* ============================
 * 检测是否到桌边（防掉桌）
 * 注意：输出逻辑因桌面颜色/模块而异，以下两种情况二选一：
 *
 * 情况A（默认）：桌面上=低电平(0)，桌边=高电平(1)
 *   → 用 if(left_val == 1 || right_val == 1)
 *
 * 情况B：桌面上=高电平(1)，桌边=低电平(0)
 *   → 用 if(left_val == 0 || right_val == 0)
 *
 * 如果小车一上电就一直后退，说明逻辑反了，把下面的 == 1 改成 == 0。
 * ============================ */
static uint8_t is_edge_detected(void)
{
    unsigned int left_val = 0;
    unsigned int right_val = 0;
    GpioGetInputVal(WIFI_IOT_GPIO_IDX_13, &left_val);
    GpioGetInputVal(WIFI_IOT_GPIO_IDX_14, &right_val);
    if(left_val == 1 || right_val == 1)
    {
        return 1;  /* 到桌边了 */
    }
    return 0;  /* 还在桌面上 */
}
/* ============================
 * 避障动作：停车 -> 后退 -> 转向 -> 停车
 * turn_left = 1  原地左转
 * turn_left = 0  原地右转
 * ============================ */
static void avoid_obstacle(int turn_left)
{
    /* 1. 停车 */
    osMutexAcquire(mutex_id, osWaitForever);
    stm32motor_control(0, 0);
    osMutexRelease(mutex_id);
    usleep(200 * 1000);
    /* 2. 后退 */
    osMutexAcquire(mutex_id, osWaitForever);
    stm32motor_control(-AVOID_SPEED, -AVOID_SPEED);
    osMutexRelease(mutex_id);
    usleep(BACKUP_MS * 1000);
    /* 3. 原地转向：左弧遇障左转，右弧遇障右转 */
    osMutexAcquire(mutex_id, osWaitForever);
    if(turn_left)
    {
        stm32motor_control(-AVOID_SPEED, AVOID_SPEED);  /* 左转 */
    }
    else
    {
        stm32motor_control(AVOID_SPEED, -AVOID_SPEED);  /* 右转 */
    }
    osMutexRelease(mutex_id);
    usleep(TURN_MS * 1000);
    /* 4. 停车，恢复 8 字循环 */
    osMutexAcquire(mutex_id, osWaitForever);
    stm32motor_control(0, 0);
    osMutexRelease(mutex_id);
    usleep(100 * 1000);
}
/*****直行任务*****/
/*
 * 一直直行，遇到障碍/桌边时执行避障（停车→后退→转向→恢复直行）。
 *
 * 调参：
 *  - STRAIGHT_SPEED 直行速度（转速×100，0~255，受 SPEED_MAX 限幅）
 *  - AVOID_TURN_DIR 遇障转向方向：1=左转，0=右转
 */
#define STRAIGHT_SPEED    100   /* 直行速度（转速×100） */
#define AVOID_TURN_DIR    1     /* 遇障转向：1左转 0右转 */
static void thread_8(void)
{
    float dist;
    while (1)
    {
        /* ===== 直行：左右轮同速前进 ===== */
        osMutexAcquire(mutex_id, osWaitForever);
        stm32motor_control(STRAIGHT_SPEED, STRAIGHT_SPEED);
        osMutexRelease(mutex_id);
        /* 每 50ms 测一次距，检测障碍或桌边 */
        dist = HCSR04_GetDistance_cm();
        if(dist < OBSTACLE_DIST_CM || is_edge_detected())
        {
            avoid_obstacle(AVOID_TURN_DIR);  /* 避障后恢复直行 */
        }
        usleep(TICK_MS * 1000);
    }
}
/*****任务创建*****/
static void correspondence(void)
{
    GpioInit(); // GPIO功能初始化
    /*********************通讯串口初始化*********************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RXD
    /****************串口参数****************/
    WifiIotUartAttribute uart_attr2 = {
        // 波特率：115200
        .baudRate = 115200,
        // 数据位：8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    /*********************超声波初始化*********************/
    HCSR04_Init();
    /*********************巡线传感器初始化（防掉桌）*********************/
    TrackSensor_Init();
    osThreadAttr_t attr;
    attr.attr_bits = 0U; // 设置osThreadJoin是否可以使用
    attr.cb_mem = NULL; // 控制块指针设置
    attr.cb_size = 0U; // 控制块指针大小
    attr.stack_mem = NULL; // 任务栈设置
    attr.stack_size = 1024 * 4; // 任务栈大小
    // 创建8字型任务
    attr.name = "thread_8"; // 创建任务名称
    attr.priority = 25; // 任务优先级
    if (osThreadNew((osThreadFunc_t)thread_8, NULL, &attr) == NULL)
    {
        printf("Falied to create thread_8!\n");
    }
    mutex_id = osMutexNew(NULL); // 创建互斥锁
    if (mutex_id == NULL)
    {
        printf("Falied to create Mutex!\n");
    }
}
APP_FEATURE_INIT(correspondence); // 启动任务
