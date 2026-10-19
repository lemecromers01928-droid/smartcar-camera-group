#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "race_autotune_generated.h"

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

volatile uint8_t race_run_enabled = 0;
volatile uint8_t race_capture_in_progress = 0;
volatile uint8_t race_lane_valid = 0;
volatile race_state_enum race_state = RACE_STATE_STOPPED;

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];

static uint8_t race_capture_requested = 0;
static uint32_t race_frame_id = 0;
static float race_last_mid_err = 0.0f;

static const char* race_state_text(race_state_enum state)
{
    switch (state)
    {
        case RACE_STATE_NORMAL:    return "NORM";
        case RACE_STATE_CURVE:     return "CURV";
        case RACE_STATE_LINE_LOST: return "LOST";
        case RACE_STATE_CAPTURE:   return "CAPT";
        default:                   return "STOP";
    }
}

static void race_dump_raw_pgm(const uint8_t* pixels)
{
    char header[96];
    int length;

    race_run_enabled = 0;
    race_capture_in_progress = 1;
    race_state = RACE_STATE_CAPTURE;
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
    tft180_show_string(0, 110, "CAPT");
    system_delay_ms(20);

    length = sprintf(header, "@PGM_BEGIN,w=%u,h=%u,bytes=%u\r\n",
                     (unsigned int)MT9V03X_W,
                     (unsigned int)MT9V03X_H,
                     (unsigned int)(MT9V03X_W * MT9V03X_H));
    debug_send_buffer((const uint8_t*)header, (uint32_t)length);
    length = sprintf(header, "P5\n%u %u\n255\n",
                     (unsigned int)MT9V03X_W,
                     (unsigned int)MT9V03X_H);
    debug_send_buffer((const uint8_t*)header, (uint32_t)length);
    debug_send_buffer(pixels, (uint32_t)(MT9V03X_W * MT9V03X_H));
    debug_send_buffer((const uint8_t*)"\r\n@PGM_END\r\n", 14U);

    race_capture_in_progress = 0;
    race_state = RACE_STATE_STOPPED;
}

static void race_handle_keys(void)
{
    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
        race_run_enabled = (uint8_t)!race_run_enabled;
        if (!race_run_enabled)
        {
            race_state = RACE_STATE_STOPPED;
            speed_target = 0.0f;
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
        }
    }
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        race_run_enabled = 0;
        race_capture_requested = 1;
    }
}

static void race_send_telemetry(uint8_t threshold,
                                int16_t mid_err,
                                int16_t servo_duty,
                                uint8_t sample_row)
{
#if RACE_TELEMETRY_ENABLE
    if ((race_frame_id % RACE_TELEMETRY_FRAME_PERIOD) == 0U)
    {
        printf("@RACE,frame=%lu,state=%u,thr=%u,err=%d,servo=%d,target=%d,enc_l=%d,enc_r=%d,left=%u,right=%u,valid=%u\r\n",
               (unsigned long)race_frame_id,
               (unsigned int)race_state,
               (unsigned int)threshold,
               (int)mid_err,
               (int)servo_duty,
               (int)speed_target,
               (int)encoder_data_1,
               (int)encoder_data_2,
               (unsigned int)left_line[sample_row],
               (unsigned int)right_line[sample_row],
               (unsigned int)race_lane_valid);
    }
#else
    (void)threshold;
    (void)mid_err;
    (void)servo_duty;
    (void)sample_row;
#endif
}

void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));

    // S1: 停车并输出一帧原始PGM；S2: 运行/停车。上电默认停车。
    key_init(10);
    tft180_show_string(0, 110, "STOP");

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
    pit_ms_init(PIT_CH0, 10);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    Init();
    interrupt_global_enable(0);

    while (1)
    {
        race_handle_keys();
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            if (race_capture_requested)
            {
                race_capture_requested = 0;
                race_dump_raw_pgm((const uint8_t*)image);
            }
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            int threshold_value = (int)get_threshold_otsu(hist_gram) + (int)RACE_THRESHOLD_OFFSET;
            unsigned char threshold;
            uint8_t sample_row;
            float mid_err;
            float steer_delta;
            float tmp_duty;

            threshold_value = MAX(0, MIN(255, threshold_value));
            threshold = (unsigned char)threshold_value;
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);

            sample_row = (RACE_STEER_SAMPLE_ROW_FROM_BOTTOM < MT9V03X_H)
                       ? (uint8_t)(MT9V03X_H - RACE_STEER_SAMPLE_ROW_FROM_BOTTOM)
                       : (uint8_t)(MT9V03X_H - 1U);
            race_lane_valid = lane_is_valid_at_row(left_line, right_line, sample_row, MT9V03X_W);

            mid_err = (float)(MT9V03X_W / 2) - (float)mid_line[sample_row];
            if (race_lane_valid && race_run_enabled)
            {
                steer_delta = RACE_STEER_KP * mid_err
                            + RACE_STEER_KD * (mid_err - race_last_mid_err);
                tmp_duty = servo_motor_duty_middle - steer_delta * SERVO_DIR;
                tmp_duty = MAX(tmp_duty, MIN(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
                tmp_duty = MIN(tmp_duty, MAX(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
                pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
                speed_target = (mid_err > RACE_CURVE_ERROR_THRESHOLD_PX ||
                                mid_err < -RACE_CURVE_ERROR_THRESHOLD_PX)
                             ? RACE_SPEED_TARGET_CURVE
                             : RACE_SPEED_TARGET_STRAIGHT;
                race_state = (speed_target == RACE_SPEED_TARGET_CURVE)
                           ? RACE_STATE_CURVE
                           : RACE_STATE_NORMAL;
                race_last_mid_err = mid_err;
            }
            else if (race_lane_valid)
            {
                /* STOP must be mechanically quiet as well as motor-safe. */
                speed_target = 0.0f;
                tmp_duty = servo_motor_duty_middle;
                pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
                race_state = RACE_STATE_STOPPED;
                race_last_mid_err = mid_err;
            }
            else
            {
                speed_target = 0.0f;
                tmp_duty = servo_motor_duty_middle;
                pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
                race_state = race_run_enabled ? RACE_STATE_LINE_LOST : RACE_STATE_STOPPED;
            }

            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][mid_line[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 100);

            // 显示关键信息
            tft180_show_int(0, 0, encoder_data_1, 3);
            tft180_show_int(0, 30, encoder_data_2, 3);
            tft180_show_int(0, 50, (int32)mid_err, 3);
            tft180_show_uint(0, 70, threshold, 3);
            tft180_show_string(0, 110, race_state_text(race_state));

            ++race_frame_id;
            race_send_telemetry(threshold, (int16_t)mid_err, (int16_t)tmp_duty, sample_row);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
    }
}
