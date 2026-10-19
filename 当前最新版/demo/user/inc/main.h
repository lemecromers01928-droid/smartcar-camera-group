#include "zf_common_headfile.h"
#include "img_process.h"

typedef enum
{
    RACE_STATE_STOPPED = 0,
    RACE_STATE_NORMAL,
    RACE_STATE_CURVE,
    RACE_STATE_LINE_LOST,
    RACE_STATE_CAPTURE
} race_state_enum;

extern volatile uint8_t race_run_enabled;
extern volatile uint8_t race_capture_in_progress;
extern volatile uint8_t race_lane_valid;
extern volatile race_state_enum race_state;
extern const float servo_motor_duty_middle;

// 编码器引脚信息
// 请确保两轮编码器前进时回传值都为正数，若回传为负请isr.c中encoder_get_count前修改正负号
// 左轮编码器
#define ENCODER_1 (QTIMER1_ENCODER1)
#define ENCODER_1_A (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B (QTIMER1_ENCODER1_CH2_C1)
// 右轮编码器
#define ENCODER_2 (QTIMER1_ENCODER2)
#define ENCODER_2_A (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B (QTIMER1_ENCODER2_CH2_C24)

// 电机参数设置
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!
// 左轮电机
#define MOTOR1_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR1_DIR (C7)
// 左电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR1_FORWARD_DIR_LEVEL (GPIO_HIGH)
// 右轮电机
#define MOTOR2_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C9)
// 右电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR2_FORWARD_DIR_LEVEL (GPIO_HIGH)


// 舵机参数设置
#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30) // 定义主板上舵机对应引脚
/* 主板有两个舵机输出口（C30 / C31），下面这个备用口和 C30 同时输出，
 * 这样不管舵机线插在哪个口上都能动。 */
#define SERVO_MOTOR_PWM_ALT (PWM4_MODULE3_CHA_C31)
#define SERVO_MOTOR_FREQ (50)                  // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (112)  // 实测机械极限：x 加到 112 顶死（2026-10-19 标定）
#define SERVO_MOTOR_R_MAX (70)   // 实测机械极限：x 减到 70 顶死（2026-10-19 标定）
/* 中值微调：上电时前轮不在正前方就调这里。每个 1.0 约 1 度，
 * 车轮偏左就减（往右修），车轮偏右就加。范围要保持在 70~112 之间。 */
#define SERVO_MOTOR_CENTER_TRIM (0.0f)
#define SERVO_DIR (SERVO_MOTOR_L_MAX>SERVO_MOTOR_R_MAX?-1.f:1.f) // 根据左右duty的大小自动决定舵机方向

// 速度指令斜率：每 10ms 允许变化的占空比，防止起步/换速时猛冲一脚
#define RACE_SPEED_SLEW         (300.0f)
// 丢线后允许继续跑的帧数：50fps 下 40 帧约 0.8 秒，超过才断电机停车
#define RACE_LANE_LOST_LIMIT    (40)

// 白带和蓝地胶的灰度差很小，Otsu 会飘。等实测出两边的灰度后可以把这里打开用固定阈值
#define RACE_THRESHOLD_FIXED_ENABLE (0)
#define RACE_THRESHOLD_FIXED        (200)

// 以下宏在初步测试例程时不可更改，后面若需更改舵机算法可以自行更改
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif
