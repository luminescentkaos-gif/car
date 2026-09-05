#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"
#include <stdint.h>

osMutexId_t mutex_id;
uint8_t flag;      //舵机旋转角度标志位

//查阅小车原理图可知，SG90舵机通过GPIO2与3861连接
//SG90舵机的控制需要MCU产生一个周期为20ms的脉冲信号，以0.5ms到2.5ms的高电平来控制舵机转动的角度
//输出20000微秒的脉冲信号(x微秒高电平,20000‑x微秒低电平)
void set_angle( unsigned int duty) {
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);//设置GPIO2为输出模式

    //GPIO2输出x微秒高电平
    GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);

    //GPIO2输出20000‑x微秒低电平
    GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/* 前置声明线程函数，避免在创建线程时隐式声明 */
static void thread1(void *arg);
static void thread2(void *arg);
static void thread3(void *arg);
static void thread_shake(void *arg);

/*
1、依据角度与脉冲的关系，设置高电平时间为500微秒，控制舵机旋转0度。
2、发送10次脉冲信号。
*/
void engine_run_0(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(500);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为1000微秒,控制舵机旋转45度。
2、发送10次脉冲信号。
*/
void engine_run_45(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(1000);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为1500微秒,控制舵机旋转90度。
2、发送10次脉冲信号。
*/
void engine_run_90(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(1500);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为2000微秒,控制舵机旋转135度。
2、发送10次脉冲信号。
*/
void engine_run_135(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(2000);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为2500微秒,控制舵机向右旋转180度。
2、发送10次脉冲信号。
*/
void engine_run_180(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(2500);
    }
}

/*****任务创建*****/
static void SG90(void)
{
    GpioInit();//初始化GPIO
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2,WIFI_IOT_IO_FUNC_GPIO_2_GPIO);   //设置GPIO模式
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2,WIFI_IOT_GPIO_DIR_OUT);         //设置为输出模式

    osThreadAttr_t  attr;
    /* 先创建互斥锁，确保线程启动后能安全使用 */
    mutex_id  =  osMutexNew(NULL);//创建互斥锁
    if (mutex_id  ==  NULL)
    {
        printf("Falied to create Mutex!\n");
    }
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.name = "thread1";
    attr.priority = 26;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL)
    {
        printf("Falied to create thread1!\n");
    }
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL)
    {
        printf("Falied to create thread2!\n");
    }
    attr.name = "thread3";
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL)
    {
        printf("Falied to create thread3!\n");
    }
    attr.name = "thread_shake";
    /* 提升摇头线程优先级，确保其能获取互斥锁持续控制舵机 */
    attr.priority = 27;
    if (osThreadNew((osThreadFunc_t)thread_shake, NULL, &attr) == NULL)
    {
        printf("Falied to create thread_shake!\n");
    }
}


/* 摇头任务：在一定范围内左右摆动舵机 */
static void thread_shake(void *arg)
{
    (void)arg;
    // 启动延时，等待其他线程初始化
    osDelay(1000U);

    // 取得互斥锁并持续持有，以便持续控制舵机（实现一直摇头）
    osMutexAcquire(mutex_id, osWaitForever);

    while (1)
    {
        printf("thread_shake is running.\r\n");
        // 向左摆动
        for (int i = 0; i < 10; i++) {
            set_angle(1000); // 左侧约45度
            osDelay(20U);
        }

        // 向右摆动
        for (int i = 0; i < 10; i++) {
            set_angle(2000); // 右侧约135度
            osDelay(20U);
        }
    }
    /* 不释放互斥锁（设计为摇头占有舵机控制） */
}

APP_FEATURE_INIT(SG90);  //启动任务