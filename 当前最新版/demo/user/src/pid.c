#include "pid.h"
#include "race_autotune_generated.h"

// 电机PID初始化参数
float speed_KP = RACE_SPEED_KP, speed_KI = RACE_SPEED_KI, speed_KD = RACE_SPEED_KD;
float speed_IMAX = 5000.0, speed_OUTMAX = 5000.0;
/* 速度环前馈（现场手调，PWM 满量程 10000）：只有比例项时，静止占空比最多
 * Kp*target，电机推不动车，必须给一个基础占空比让车起步并维持速度。
 * 架空后转速到不了 30 就往上加（3000/3500），明显冲过头就往下减。
 * 注意：这个值不放进 race_autotune_generated.h，避免赛道自优化工具重新生成时丢掉。 */
float speed_feedforward = 2500.0f;
float speed_target = RACE_SPEED_TARGET_STRAIGHT;
float speed_real = 0.0;
float speed_pwm = 0.0;
float speed_pwm_l = 0.0;
float speed_pwm_r = 0.0;

pid_param_t speed_pid_l;  // 电机PID
pid_param_t speed_pid_r;  // 电机PID

/*******************************************************************************
* 函 数 名         : My_Pid_Init
* 函数功能         : PID初始化	
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_Pid_Init(void)
{
    Pid_Param_Init(&speed_pid_l, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
    Pid_Param_Init(&speed_pid_r, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
}


/*******************************************************************************
* 函 数 名         : Pid_Param_Init
* 函数功能         : PID参数初始化	
* 输    入         : imax:积分项最大值
* 输    出         : 无
*******************************************************************************/
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->imax      = imax;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->outmax    = outmax;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*******************************************************************************
* 函 数 名         : PidLocCtrl
* 函数功能	   : 位置式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error, float t)
{
    if (t <= 0.0f)
        t = 1.0f;
    /* imax limits the stored integral. The old implementation limited a
     * temporary value and then overwrote it, so wind-up was never prevented. */
    pid->integrator += error * t;
    pid->integrator = constrain_float(pid->integrator, -pid->imax, pid->imax);
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd/t * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);

    return pid->out;
}


/*******************************************************************************
* 函 数 名         : PidIncCtrl
* 函数功能	   : 增量式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error, float t)
{
    if (t <= 0.0f)
        t = 1.0f;

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error * t ;
    pid->out_d = pid->kd/t * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);
    return pid->out;
}

/*******************************************************************************
* 函 数 名         : constrain_float
* 函数功能         : 浮点型数限幅
* 输    入         : amt,low,high
* 输    出         : float
*******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*******************************************************************************
* 函 数 名         : constrain_short
* 函数功能         : 短整型数限幅
* 输    入         : amt,low,high
* 输    出         : short
*******************************************************************************/
short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

