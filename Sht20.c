#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_sht20.h"

osSemaphoreId_t sem1;

static void thread1(void *argument);
static void thread2(void *argument);
static void thread3(void *argument);

static void i2c_sht20_demo(void)
{
    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    sem1 = osSemaphoreNew(4, 0, NULL);
    if (sem1 == NULL)
    {
        printf("Failed to create Semaphore1!\n");
        return;
    }

    attr.name = "thread1";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL)
    {
        printf("Failed to create thread1!\n");
    }

    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL)
    {
        printf("Failed to create thread2!\n");
    }

    attr.name = "thread3";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL)
    {
        printf("Failed to create thread3!\n");
    }

}

static void thread1(void *argument)
{
    (void)argument;
    while (1)
    {
        osSemaphoreRelease(sem1);
        osSemaphoreRelease(sem1);
        printf("\n");
        printf("Thread1 release energy!\n");
        osDelay(300);
    }
}

static void thread2(void *argument)
{
    (void)argument;
    float temperature = 0, humidity = 0;
    printf("i2c_sht20_demo()!");
    SHT20_Init();
    while (1)
    {
        osSemaphoreAcquire(sem1, osWaitForever);
        SHT20_ReadData(&temperature, &humidity);
        printf("temperature = %.2f    humidity = %.2f\r\n", temperature, humidity);
        printf("Thread2 get energy!\n");
        osDelay(1);
    }
}

static void thread3(void *argument)
{
    (void)argument;
    while (1)
    {
        osSemaphoreAcquire(sem1, osWaitForever);
        printf("Thread3 get energy!\n");
        osDelay(1);
    }
}

APP_FEATURE_INIT(i2c_sht20_demo);
