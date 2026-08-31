#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "ohos_init.h"
#include "cmsis_os2.h"

#include "hal_bsp_ap3216c.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ssd1306.h"

static void oled_show_sensor(uint16_t ir, uint16_t als, uint16_t ps, float temp, float humi)
{
    char buf[32];

    SSD1306_CLS();

    snprintf(buf, sizeof(buf), "IR:%u", ir);
    SSD1306_ShowStr(0, 0, (uint8_t *)buf, 16);

    snprintf(buf, sizeof(buf), "ALS:%u", als);
    SSD1306_ShowStr(0, 1, (uint8_t *)buf, 16);

    snprintf(buf, sizeof(buf), "PS:%u", ps);
    SSD1306_ShowStr(0, 2, (uint8_t *)buf, 16);

    snprintf(buf, sizeof(buf), "T:%.1f", temp);
    SSD1306_ShowStr(0, 3, (uint8_t *)buf, 16);

    snprintf(buf, sizeof(buf), "H:%.1f", humi);
    SSD1306_ShowStr(64, 3, (uint8_t *)buf, 16);
}

static void sensor_task(void *argument)
{
    (void)argument;

    uint16_t ir = 0;
    uint16_t als = 0;
    uint16_t ps = 0;
    float temp = 0.0f;
    float humi = 0.0f;

    if (AP3216C_Init() != 0) {
        printf("AP3216C init failed!\r\n");
        return;
    }

    if (SHT20_Init() != 0) {
        printf("SHT20 init failed!\r\n");
        return;
    }

    if (SSD1306_Init() != 0) {
        printf("SSD1306 init failed!\r\n");
        return;
    }

    printf("AP3216C and SHT20 init success!\r\n");

    while (1) {
        if (AP3216C_ReadData(&ir, &als, &ps) == 0) {
            printf("IR=%u\r\n", ir);
            printf("ALS=%u\r\n", als);
            printf("PS=%u\r\n", ps);
        }

        if (SHT20_ReadData(&temp, &humi) == 0) {
            printf("TEMP=%.2f\r\n", temp);
            printf("HUMI=%.2f\r\n", humi);
        }

        oled_show_sensor(ir, als, ps, temp, humi);
        sleep(1);
    }
}

static void sensor_demo(void)
{
    osThreadAttr_t attr;
    attr.name = "sensor_task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)sensor_task, NULL, &attr) == NULL) {
        printf("Failed to create sensor task!\r\n");
    }
}

APP_FEATURE_INIT(sensor_demo);
