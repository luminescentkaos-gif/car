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
static void Servo_Init(void);
static void Servo_SetAngle(unsigned int pulse_us);
static float HCSR04_ScanAngle(unsigned int pulse_us);
static void InfraredObstacle_Init(void);
static uint8_t get_ir_obstacle_mask(void);
static void avoid_obstacle(int turn_left, uint8_t back_up);
uint8_t uart_sendbuf[20];
osMutexId_t mutex_id;
static uint8_t infrared_ready = 0;
static volatile uint8_t servo_ir_mask = 0;
/* 发送速度上限：电机实际转速的一百倍（0~255，超出会被截断，默认150=1.5rad/s） */
#define SPEED_MAX    150
/* ============================
 * 超声波避障 + 黑色胶布检测
 *
 * 避障：HC-SR04 朝前；两个红外传感器朝下检测地面黑色胶布。
 * 黑色胶布通常反射弱，传感器输出低电平(0)。
 * ============================ */
#define OBSTACLE_DIST_CM   8   /* 超声波小于此距离判定为有障碍 */
#define AVOID_SPEED        100   /* 避障动作速度（转速×100） */
#define TURN_SPEED         150   /* 原地转弯速度（转速×100） */
#define BACKUP_MS          350   /* 首次遇障后退持续时间(ms) */
#define TURN_MS            450   /* 后退后立即原地转向持续时间(ms) */
#define CLEAR_CONFIRM_CYCLES 3   /* 转弯后连续确认安全的次数 */
#define TICK_MS            50    /* 测距时间片(ms)，越小反应越快 */
#define TAPE_CONFIRM_CYCLES 1    /* 检测到一次高电平立即避让，避免漏掉窄胶带 */
#define SERVO_LEFT_US      1000  /* 舵机左侧角度 */
#define SERVO_CENTER_US    1500  /* 舵机中间角度 */
#define SERVO_RIGHT_US     2000  /* 舵机右侧角度 */
#define SERVO_SETTLE_CYCLES 12  /* 舵机转到测量位置后的保持周期，约240ms */
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

    /* ===== 主动微小振动：左右轮同步加/减，让车身高频微抖 ===== */
    #define VIBRATE_AMP  15    /* 振动幅度，越大抖得越厉害，从10开始试 */
    static uint8_t vibrate_flag = 0;
    vibrate_flag = !vibrate_flag;
    if(vibrate_flag)
    {
        motorA += VIBRATE_AMP;
        motorB += VIBRATE_AMP;
        if(motorA > SPEED_MAX) motorA = SPEED_MAX;
        if(motorB > SPEED_MAX) motorB = SPEED_MAX;
    }
    /* ===== 振动结束 ===== */

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
 * 舵机扫描
 * SG90 信号线 -> GPIO_2，500~2500us 对应舵机角度范围
 * ============================ */
static void Servo_Init(void)
{
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_2, WIFI_IOT_GPIO_DIR_OUT);
}
static void Servo_SetAngle(unsigned int pulse_us)
{
    GpioSetOutputVal(WIFI_IOT_GPIO_IDX_2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(pulse_us);
    GpioSetOutputVal(WIFI_IOT_GPIO_IDX_2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - pulse_us);
}
static void Servo_MoveTo(unsigned int pulse_us)
{
    int i;
    for (i = 0; i < SERVO_SETTLE_CYCLES; i++)
    {
        Servo_SetAngle(pulse_us);
        if (infrared_ready)
        {
            servo_ir_mask = get_ir_obstacle_mask();
            if (servo_ir_mask != 0)
            {
                break;
            }
        }
    }
}
static float HCSR04_ScanAngle(unsigned int pulse_us)
{
    float distance;
    Servo_MoveTo(pulse_us);
    distance = HCSR04_GetDistance_cm();
    if (distance < 0.0f)
    {
        return 400.0f;  /* 无回波按远距离处理 */
    }
    return distance;
}
static void Servo_SelfTest(void)
{
    Servo_MoveTo(SERVO_CENTER_US);
    Servo_MoveTo(SERVO_LEFT_US);
    Servo_MoveTo(SERVO_RIGHT_US);
    Servo_MoveTo(SERVO_CENTER_US);
}
/* ============================
 * 地面红外传感器初始化
 * 左路 -> GPIO_13，右路 -> GPIO_14
 * 两个传感器应朝下安装，用于检测黑色胶布
 * ============================ */
static void InfraredObstacle_Init(void)
{
    // 左侧地面传感器: IO13 设为 GPIO 功能 + 输入
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_13, WIFI_IOT_GPIO_DIR_IN);
    // 右侧地面传感器: IO14 设为 GPIO 功能 + 输入
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_GPIO_IDX_14, WIFI_IOT_GPIO_DIR_IN);
    infrared_ready = 1;
}
/* 返回值：bit0=左侧检测到障碍物，bit1=右侧检测到障碍物。
 * 抖动检测：贴地时黑胶带和桌面静态都是0，但黑胶带上方颠簸抖动更多。
 */
/* 振动统计参数：在车身振动窗口内快速采样，统计高电平次数 */
#define IR_SAMPLE_TIMES       40    /* 采样次数 */
#define IR_SAMPLE_INTERVAL_US 100   /* 采样间隔，总窗口约4ms */
#define IR_WHITE_THRESHOLD    5     /* 高电平次数<=此值判定为白线（反射最强） */

/* 返回值：bit0=左侧检测到白线，bit1=右侧检测到白线。 */
static uint8_t get_ir_obstacle_mask(void)
{
    unsigned int left_val = 0;
    unsigned int right_val = 0;
    int i;
    uint8_t left_count = 0;
    uint8_t right_count = 0;
    for(i = 0; i < IR_SAMPLE_TIMES; i++)
    {
        GpioGetInputVal(WIFI_IOT_GPIO_IDX_13, &left_val);
        GpioGetInputVal(WIFI_IOT_GPIO_IDX_14, &right_val);
        if(left_val == 1) left_count++;
        if(right_val == 1) right_count++;
        hi_udelay(IR_SAMPLE_INTERVAL_US);
    }
    /* 调试：每10次打印一次高电平次数，方便调阈值 */
    {
        static int dbg_cnt = 0;
        if(++dbg_cnt >= 10)
        {
            dbg_cnt = 0;
            printf("IR vib: L=%d R=%d (white<=%d)\n", left_count, right_count, IR_WHITE_THRESHOLD);
        }
    }
    /* 白线反射最强，振动时高电平次数最少；<=阈值判定为白线 */
    return (left_count <= IR_WHITE_THRESHOLD ? 1 : 0) |
           (right_count <= IR_WHITE_THRESHOLD ? 2 : 0);
}

/* ============================
 * 避障动作：停车 -> 后退 -> 转向 -> 停车
 * turn_left = 1  原地左转
 * turn_left = 0  原地右转
 * ============================ */
static void avoid_obstacle(int turn_left, uint8_t back_up)
{
    /* 1. 首次触发时后退，连续转弯时不重复后退 */
    osMutexAcquire(mutex_id, osWaitForever);
    stm32motor_control(0, 0);
    osMutexRelease(mutex_id);
    usleep(50 * 1000);
    if (back_up)
    {
        osMutexAcquire(mutex_id, osWaitForever);
        stm32motor_control(-AVOID_SPEED, -AVOID_SPEED);
        osMutexRelease(mutex_id);
        usleep(BACKUP_MS * 1000);
    }
    /* 2. 原地转向，确保车头离开障碍物方向 */
    osMutexAcquire(mutex_id, osWaitForever);
    if(turn_left)
    {
        stm32motor_control(-TURN_SPEED, TURN_SPEED);  /* 左转 */
    }
    else
    {
        stm32motor_control(TURN_SPEED, -TURN_SPEED);  /* 右转 */
    }
    osMutexRelease(mutex_id);
    usleep(TURN_MS * 1000);
    /* 3. 停车，等待下一轮确认 */
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
#define STRAIGHT_SPEED    130   /* 直行速度（转速×100） */
#define AVOID_TURN_DIR    1     /* 遇障转向：1左转 0右转 */
static void thread_8(void)
{
    float center_distance;
    float left_distance;
    float right_distance;
    uint8_t turn_left = AVOID_TURN_DIR;
    uint8_t ir_mask;
    uint8_t ultrasonic_triggered;
    uint8_t confirmed_ir_mask;
    uint8_t left_tape_cycles = 0;
    uint8_t right_tape_cycles = 0;
    uint8_t continuous_turning = 0;
    uint8_t clear_cycles = 0;
    while (1)
    {
        /* ===== 调试：打印红外传感器原始电平 ===== */
        {
            unsigned int dbg_left = 0, dbg_right = 0;
            static int dbg_cnt = 0;
            GpioGetInputVal(WIFI_IOT_GPIO_IDX_13, &dbg_left);
            GpioGetInputVal(WIFI_IOT_GPIO_IDX_14, &dbg_right);
            if(++dbg_cnt >= 10)  /* 每10个周期(约500ms)打印一次 */
            {
                dbg_cnt = 0;
                printf("IR: L=%d R=%d\n", dbg_left, dbg_right);
            }
        }
        /* ===== 调试结束 ===== */
        /* 先读地面红外，避免舵机扫描延迟导致车辆压上胶带 */
        servo_ir_mask = 0;
        ir_mask = get_ir_obstacle_mask();
        if (ir_mask == 0)
        {
            /* 无胶带时才执行耗时的左、中、右超声波扫描 */
            center_distance = HCSR04_ScanAngle(SERVO_CENTER_US);
            ir_mask = servo_ir_mask;
            if (ir_mask == 0)
            {
                left_distance = HCSR04_ScanAngle(SERVO_LEFT_US);
                ir_mask = servo_ir_mask;
            }
            else
            {
                left_distance = 400.0f;
            }
            if (ir_mask == 0)
            {
                right_distance = HCSR04_ScanAngle(SERVO_RIGHT_US);
                ir_mask = servo_ir_mask;
            }
            else
            {
                right_distance = 400.0f;
            }
            if (ir_mask == 0)
            {
                Servo_MoveTo(SERVO_CENTER_US);
            }
        }
        else
        {
            /* 红外已发现胶带，本轮不再等待舵机扫描 */
            center_distance = 400.0f;
            left_distance = 400.0f;
            right_distance = 400.0f;
        }
        ultrasonic_triggered = (center_distance < OBSTACLE_DIST_CM) ||
            (left_distance < OBSTACLE_DIST_CM) ||
            (right_distance < OBSTACLE_DIST_CM);
        if (ir_mask & 1)
        {
            if (left_tape_cycles < TAPE_CONFIRM_CYCLES)
            {
                left_tape_cycles++;
            }
        }
        else
        {
            left_tape_cycles = 0;
        }
        if (ir_mask & 2)
        {
            if (right_tape_cycles < TAPE_CONFIRM_CYCLES)
            {
                right_tape_cycles++;
            }
        }
        else
        {
            right_tape_cycles = 0;
        }
        confirmed_ir_mask = (left_tape_cycles >= TAPE_CONFIRM_CYCLES ? 1 : 0) |
            (right_tape_cycles >= TAPE_CONFIRM_CYCLES ? 2 : 0);
        if (confirmed_ir_mask != 0 || ultrasonic_triggered)
        {
            clear_cycles = 0;
            if (!continuous_turning && confirmed_ir_mask == 1)
            {
                turn_left = 0;  /* 左侧压到黑胶布，向右转 */
            }
            else if (!continuous_turning && confirmed_ir_mask == 2)
            {
                turn_left = 1;  /* 右侧压到黑胶布，向左转 */
            }
            else if (!continuous_turning && confirmed_ir_mask == 3)
            {
                turn_left = AVOID_TURN_DIR;  /* 两侧压到黑胶布，固定向一侧转 */
            }
            else if (!continuous_turning && ultrasonic_triggered)
            {
                turn_left = (left_distance >= right_distance);  /* 向更空的一侧转 */
            }
            avoid_obstacle(turn_left, !continuous_turning);
            continuous_turning = 1;
            left_tape_cycles = 0;
            right_tape_cycles = 0;
            continue;
        }
        if (continuous_turning)
        {
            clear_cycles++;
            if (clear_cycles < CLEAR_CONFIRM_CYCLES)
            {
                osMutexAcquire(mutex_id, osWaitForever);
                stm32motor_control(0, 0);
                osMutexRelease(mutex_id);
                usleep(TICK_MS * 1000);
                continue;
            }
            continuous_turning = 0;
            clear_cycles = 0;
        }
        /* 当前方向安全，才保持前进 */
        osMutexAcquire(mutex_id, osWaitForever);
        stm32motor_control(STRAIGHT_SPEED, STRAIGHT_SPEED);
        osMutexRelease(mutex_id);
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
    /*********************舵机初始化*********************/
    Servo_Init();
    /*********************舵机启动自检：中->左->右->中*********************/
    Servo_SelfTest();
    /*********************地面红外传感器初始化*********************/
    InfraredObstacle_Init();
    mutex_id = osMutexNew(NULL); // 在线程启动前创建互斥锁
    if (mutex_id == NULL)
    {
        printf("Falied to create Mutex!\n");
        return;
    }
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
}
APP_FEATURE_INIT(correspondence); // 启动任务
