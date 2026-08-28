#include "stm32f10x.h"
#include "stdio.h"
#include "encode.h"
#include "motor.h"
#include "control_system.h"


/* ==============================
 * 全局变量
 * ============================== */

int L_coder = 0;
int R_coder = 0;

int Motor_A = 0;
int Motor_B = 0;


/* ==============================
 * 参数
 * ============================== */

/* 控制周期 100ms */
#define CONTROL_PERIOD_MS      100

/* 编码器每转计数 */
#define CPR_PER_REV            2800.0f

/*
 * 先用3.0rad/s测试
 *
 * 100ms目标约133个计数
 */
#define TARGET_SPEED_RAD       3.0f


/* 左轮PI */
#define KP_A                   3.0f
#define KI_A                   0.20f


/* 右轮PI */
#define KP_B                   3.0f
#define KI_B                   0.20f


/* 积分限幅 */
#define INTEGRAL_MAX           3000.0f
#define INTEGRAL_MIN          -3000.0f


/*
 * 左右轮启动PWM
 *
 * 现在先都设1500
 *
 * 后面根据实际启动情况再调整
 */
#define PWM_START_A            1500
#define PWM_START_B            1500


/*
 * 现在先关闭跑直纠偏
 *
 * 先把双轮速度闭环跑稳定
 */
#define STRAIGHT_K             0.0f


/* ==============================
 * rad/s转换成100ms编码器计数
 * ============================== */
int Rs_To_CPR(float speed)
{
    float period_s;
    float count;

    period_s = CONTROL_PERIOD_MS / 1000.0f;

    count =
        speed
        / (2.0f * 3.1415926f)
        * CPR_PER_REV
        * period_s;

    return (int)count;
}


/* ==============================
 * 左轮 PI
 * ============================== */
int Incremental_PI_A(int now, int target)
{
    static float integral_A = 0.0f;

    float error;
    float output;


    /* 速度误差 */
    error = (float)(target - now);


    /* 积分 */
    integral_A += error;


    /* 积分限幅 */
    if(integral_A > INTEGRAL_MAX)
        integral_A = INTEGRAL_MAX;

    if(integral_A < INTEGRAL_MIN)
        integral_A = INTEGRAL_MIN;


    /* PI */
    output =
        KP_A * error
        +
        KI_A * integral_A;


    /*
     * 仅在电机还没有开始转的时候
     * 提供启动PWM
     */
    if(target > 0)
    {
        if(now > -5 && now < 5)
        {
            if(output < PWM_START_A)
                output = PWM_START_A;
        }
    }
    else if(target < 0)
    {
        if(now > -5 && now < 5)
        {
            if(output > -PWM_START_A)
                output = -PWM_START_A;
        }
    }


    /* PWM限幅 */
    if(output > 7199)
        output = 7199;

    if(output < -7199)
        output = -7199;


    return (int)output;
}


/* ==============================
 * 右轮 PI
 * ============================== */
int Incremental_PI_B(int now, int target)
{
    static float integral_B = 0.0f;

    float error;
    float output;


    /* 速度误差 */
    error = (float)(target - now);


    /* 积分 */
    integral_B += error;


    /* 积分限幅 */
    if(integral_B > INTEGRAL_MAX)
        integral_B = INTEGRAL_MAX;

    if(integral_B < INTEGRAL_MIN)
        integral_B = INTEGRAL_MIN;


    /* PI */
    output =
        KP_B * error
        +
        KI_B * integral_B;


    /*
     * 启动PWM
     */
    if(target > 0)
    {
        if(now > -5 && now < 5)
        {
            if(output < PWM_START_B)
                output = PWM_START_B;
        }
    }
    else if(target < 0)
    {
        if(now > -5 && now < 5)
        {
            if(output > -PWM_START_B)
                output = -PWM_START_B;
        }
    }


    /* PWM限幅 */
    if(output > 7199)
        output = 7199;

    if(output < -7199)
        output = -7199;


    return (int)output;
}


/* ==============================
 * 直线速度闭环
 *
 * forward = 1  前进
 * forward = -1 后退
 * ============================== */
void Straight_Control(int forward)
{
    int encoder_left;
    int encoder_right;

    int target_count;
    int target_left;
    int target_right;

    int pwm_left;
    int pwm_right;

    int abs_left;
    int abs_right;

    int diff;
    int correction;


    /* ==========================
     * 1.读取编码器
     * ========================== */

    /*
     * 左轮：
     * 前进+
     * 后退-
     */
    encoder_left = Encoder_GetLeft();


    /*
     * 右轮：
     * 前进+
     * 后退-
     *
     * 注意：
     * 这里绝对不能取负号！
     */
    encoder_right = Encoder_GetRight();


    /* 保存 */
    L_coder = encoder_left;
    R_coder = encoder_right;


    /* ==========================
     * 2.目标编码器速度
     * ========================== */

    target_count =
        Rs_To_CPR(TARGET_SPEED_RAD);


    /* ==========================
     * 3.设置目标方向
     * ========================== */

    if(forward > 0)
    {
        /*
         * 前进
         */
        target_left  = target_count;
        target_right = target_count;
    }
    else
    {
        /*
         * 后退
         */
        target_left  = -target_count;
        target_right = -target_count;
    }


    /* ==========================
     * 4.左右轮分别PI闭环
     * ========================== */

    pwm_left =
        Incremental_PI_A(
            encoder_left,
            target_left
        );


    pwm_right =
        Incremental_PI_B(
            encoder_right,
            target_right
        );


    /* ==========================
     * 5.计算速度绝对值
     * ========================== */

    if(encoder_left < 0)
        abs_left = -encoder_left;
    else
        abs_left = encoder_left;


    if(encoder_right < 0)
        abs_right = -encoder_right;
    else
        abs_right = encoder_right;


    /* ==========================
     * 6.左右轮速度差
     * ========================== */

    diff =
        abs_left - abs_right;


    /* ==========================
     * 7.直线纠偏
     * ========================== */

    correction =
        (int)(STRAIGHT_K * (float)diff);


    /*
     * 左轮快：
     * 左PWM减小
     * 右PWM增大
     *
     * 右轮快：
     * 左PWM增大
     * 右PWM减小
     */
    pwm_left  -= correction;
    pwm_right += correction;


    /* ==========================
     * 8.PWM限幅
     * ========================== */

    if(pwm_left > 7199)
        pwm_left = 7199;

    if(pwm_left < -7199)
        pwm_left = -7199;


    if(pwm_right > 7199)
        pwm_right = 7199;

    if(pwm_right < -7199)
        pwm_right = -7199;


    /* ==========================
     * 9.输出PWM
     * ========================== */

    Motor_A = pwm_left;
    Motor_B = pwm_right;

    Set_Pwm(
        Motor_A,
        Motor_B
    );


    /* ==========================
     * 10.串口输出
     * ========================== */

    printf(
        "L:%d R:%d\r\n",
        encoder_left,
        encoder_right
    );

    printf(
        "TL:%d TR:%d\r\n",
        target_left,
        target_right
    );

    printf(
        "PWM_L:%d PWM_R:%d\r\n",
        Motor_A,
        Motor_B
    );

    printf(
        "DIFF:%d COR:%d\r\n",
        diff,
        correction
    );
}

