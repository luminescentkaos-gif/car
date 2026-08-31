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
 
 //参数设置区域
 
#define CONTROL_PERIOD_MS      100
#define CPR_PER_REV            2800.0f
#define TARGET_SPEED_RAD       3.0f

#define KP_A                   1.0f
#define KI_A                   0.05f

#define KP_B                   1.0f
#define KI_B                   0.05f

#define INTEGRAL_MAX           2000.0f
#define INTEGRAL_MIN          -2000.0f

#define STRAIGHT_K             0.2f

/* 直线闭环参数 */
#define STRAIGHT_FF_K  13.0f   /* 从 17 降下来，减少起步超速 */
#define STRAIGHT_KP    2.0f    /* 从 1.0 加大，比例项直接压制超速 */
#define STRAIGHT_KI    0.08f   /* 略加大，加快收敛 */
#define STRAIGHT_DEADZONE 3
#define PWM_MAX        5000


/* 直线纠偏参数 */
#define STEER_K        0.05f   /* 里程差纠偏增益，先小后大 */
#define STEER_MAX      400     /* 单次纠偏量限幅，防蛇形摆动 */
#define DIST_MAX       800     /* 里程差累计限幅，防饱和 */



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
 * 左右轮 PI
 * ============================== */
int Incremental_PI_A(int now, int target)
{
    static float integral_A = 0.0f;
    float error;
    float output;

    error = (float)(target - now);
    integral_A += error;

    if(integral_A > INTEGRAL_MAX)
        integral_A = INTEGRAL_MAX;
    if(integral_A < INTEGRAL_MIN)
        integral_A = INTEGRAL_MIN;

    output = KP_A * error + KI_A * integral_A;

    if(output > 7199)
        output = 7199;
    if(output < -7199)
        output = -7199;

    return (int)output;
}

int Incremental_PI_B(int now, int target)
{
    static float integral_B = 0.0f;
    float error;
    float output;

    error = (float)(target - now);
    integral_B += error;

    if(integral_B > INTEGRAL_MAX)
        integral_B = INTEGRAL_MAX;
    if(integral_B < INTEGRAL_MIN)
        integral_B = INTEGRAL_MIN;

    output = KP_B * error + KI_B * integral_B;

    if(output > 7199)
        output = 7199;
    if(output < -7199)
        output = -7199;

    return (int)output;
}

/* 前馈+比例 闭环，带死区与限幅。target/now 均为每周期编码器计数 */
static int Wheel_Control(int target, int now, int forward)
{
    static float integral = 0.0f;
    int error = target - now;
    int output;

    /* 前馈：起步就有足够动力，PWM = FF_K × 目标计数（带符号） */
    output = (int)((float)target * STRAIGHT_FF_K);

    if (error > -STRAIGHT_DEADZONE && error < STRAIGHT_DEADZONE) {
        integral = 0.0f;
        return output;          /* 死区内保持前馈动力，清积分 */
    }

    /* 积分补偿稳态误差 */
    integral += (float)error * STRAIGHT_KI;
    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
    if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

    output += (int)((float)error * STRAIGHT_KP) + (int)integral;
    if (output >  PWM_MAX) output =  PWM_MAX;
    if (output < -PWM_MAX) output = -PWM_MAX;
    return output;
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
	    int steer;                    /* 纠偏量 */
    static long dist_error = 0;   /* 左右轮里程差累计：正 = 左轮多走 */
	    static int dbg_cnt = 0;   /* 调试打印分频计数 */



    encoder_left  = (int)Encoder_GetLeft();
    encoder_right = (int)Encoder_GetRight();
    L_coder = encoder_left;    /* ← 加这两行，把实际值赋给打印用的全局变量 */
    R_coder = encoder_right;

    /* 目标：每周期增量计数，与 Rs_To_CPR 同口径 */
    target_count = Rs_To_CPR(TARGET_SPEED_RAD);
    if (forward > 0)
    {
        target_left  =  target_count;
        target_right =  target_count;
    }
    else
    {
        target_left  = -target_count;
        target_right = -target_count;
    }

    pwm_left  = Wheel_Control(target_left,  encoder_left,  forward);
    pwm_right = Wheel_Control(target_right, encoder_right, forward);

    /* ===== 直线纠偏：里程差 → 反向差速补偿 ===== */
    dist_error += (encoder_left - encoder_right);      /* 累计里程差 */
    if (dist_error >  DIST_MAX) dist_error =  DIST_MAX;
    if (dist_error < -DIST_MAX) dist_error = -DIST_MAX;
    steer = (int)((float)dist_error * STEER_K);
    if (steer >  STEER_MAX) steer =  STEER_MAX;
    if (steer < -STEER_MAX) steer = -STEER_MAX;
    pwm_left  -= steer;   /* 左轮多走 → 左轮减速 */
    pwm_right += steer;   /* 右轮加速 */

    Motor_A = pwm_left;
    Motor_B = pwm_right;
    Set_Pwm(Motor_A, Motor_B);
		
		    /* ===== 串口调试输出：每10个周期(约1s)打印一次 ===== */
    if (++dbg_cnt >= 10)
    {
        dbg_cnt = 0;
        printf("tgt=%d L=%d R=%d PWM_A=%d PWM_B=%d steer=%d dist=%ld\r\n",
               target_count, L_coder, R_coder, Motor_A, Motor_B, steer, dist_error);
    }


}

