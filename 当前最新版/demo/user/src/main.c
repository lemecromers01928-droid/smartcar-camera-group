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

// 舵机动作角度中值（左右极限的中点，再加一点微调）
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f
                                    + SERVO_MOTOR_CENTER_TRIM;

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
static uint8_t race_lane_state = LANE_NONE;     // 采样行的边线状态
static uint8_t race_mid_col = MT9V03X_W / 2;    // 采样行补线后的中线列号
static uint8_t race_lane_lost_count = 0;        // 连续丢线帧计数

static uint8_t race_servo_test = 0;             // 舵机自检模式
static float   race_servo_test_x = 0.0f;        // 自检时当前的舵机 x 值
static int8_t  race_servo_test_dir = 1;         // 自检时的步进方向
static uint8_t race_key1_long_latch = 0;        // 长按只触发一次

#define RACE_SAMPLE_ROW_COUNT   (5U)
static const uint8_t  race_sample_rows[RACE_SAMPLE_ROW_COUNT] = {90U, 80U, 70U, 60U, 50U};
static uint16_t       race_row_width[RACE_SAMPLE_ROW_COUNT]   = {150U, 140U, 130U, 120U, 110U};
static uint8_t        race_used_row = 0xFFU;    // 实际采用的行序号，0xFF 表示全部失败

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

/* 舵机统一从这两个口同时输出：主板有 C30 / C31 两个舵机口，
 * 只写其中一个时，插错口的表现就是"舵机完全不动"。 */
static void servo_write(float x)
{
    pwm_set_duty(SERVO_MOTOR_PWM,     SERVO_MOTOR_DUTY(x));
    pwm_set_duty(SERVO_MOTOR_PWM_ALT, SERVO_MOTOR_DUTY(x));
}

static void race_dump_raw_pgm(const uint8_t* pixels)
{
    char header[96];
    int length;

    race_run_enabled = 0;
    race_capture_in_progress = 1;
    race_state = RACE_STATE_CAPTURE;
    servo_write(servo_motor_duty_middle);
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
    /* 长按"运行键"进出舵机自检：只依赖板上那一个确认能用的键。
     * 自检里短按运行键 = x 走 2°，走到 130 自动掉头往 55，再走到 55 又掉头，
     * 这样按着一个键就能把整个行程走一遍，屏幕左上实时显示 x。
     * 判断方法：x 变大时前轮往左偏 = 方向对；顶到机械限位会听到舵机嗡嗡响。 */
    if (KEY_LONG_PRESS == key_get_state(KEY_1))
    {
        if (!race_key1_long_latch)
        {
            race_key1_long_latch = 1;
            race_servo_test = (uint8_t)(!race_servo_test);
            race_run_enabled = 0;
            speed_target = 0.0f;
            race_servo_test_dir = 1;
            race_servo_test_x = servo_motor_duty_middle;
            if (race_servo_test)
            {
                tft180_show_string(0, 110, "SVO");
                /* 进场先明显摆动一次（左15度 → 停1秒 → 右15度 → 停1秒 → 回中），
                 * 用来确认舵机到底有没有响应：有响应一定会看到前轮大幅摆动。 */
                servo_write(servo_motor_duty_middle + 15.0f);
                system_delay_ms(600U);
                servo_write(servo_motor_duty_middle - 15.0f);
                system_delay_ms(600U);
                servo_write(race_servo_test_x);
            }
            else
            {
                servo_write(servo_motor_duty_middle);
                tft180_show_string(0, 110, "STOP");
            }
        }
        return;
    }
    race_key1_long_latch = 0;

    if (race_servo_test)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_1))
        {
            key_clear_state(KEY_1);
            race_servo_test_x += 2.0f * (float)race_servo_test_dir;
            if (race_servo_test_x >= 130.0f)
            {
                race_servo_test_x = 130.0f;
                race_servo_test_dir = -1;
            }
            if (race_servo_test_x <= 55.0f)
            {
                race_servo_test_x = 55.0f;
                race_servo_test_dir = 1;
            }
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_2))
        {
            /* 另一个键用于原地反向，方便贴着机械限位来回微调 */
            key_clear_state(KEY_2);
            race_servo_test_dir = (int8_t)(-race_servo_test_dir);
        }
        servo_write(race_servo_test_x);
        tft180_show_int(0, 0, (int32)race_servo_test_x, 3);
        return;
    }

    /* 短按同一个键 = 运行/停车（这个键已经被实车验证过） */
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        race_run_enabled = (uint8_t)!race_run_enabled;
        if (!race_run_enabled)
        {
            race_state = RACE_STATE_STOPPED;
            speed_target = 0.0f;
            race_lane_lost_count = 0;
            servo_write(servo_motor_duty_middle);
        }
    }
    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
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

    // 舵机：主板两个舵机口（C30/C31）同时初始化并输出，插哪个都能动
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR_PWM_ALT, SERVO_MOTOR_FREQ, 0);
    servo_write(servo_motor_duty_middle);

    // 物理S1: 停车并输出一帧原始PGM；物理S2: 运行/停车。上电默认停车。
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
        if (race_servo_test)
        {
            /* 自检模式：不处理图像，舵机由按键控制，电机保持停止 */
            mt9v03x_finish_flag = 0;
            continue;
        }
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            /* 二值化之前先记下两个采样点的灰度：左边大概率是蓝地胶，中间大概率是白带。
             * 这两个数决定阈值该取多少，是当前最需要的现场数据。 */
            uint8_t gray_left  = image[race_sample_rows[0]][5U];
            uint8_t gray_center = image[race_sample_rows[0]][MT9V03X_W / 2];
            if (race_capture_requested)
            {
                race_capture_requested = 0;
                race_dump_raw_pgm((const uint8_t*)image);
            }
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
#if RACE_THRESHOLD_FIXED_ENABLE
            int threshold_value = RACE_THRESHOLD_FIXED;
#else
            int threshold_value = (int)get_threshold_otsu(hist_gram) + (int)RACE_THRESHOLD_OFFSET;
#endif
            unsigned char threshold;
            uint8_t sample_row;
            float mid_err;
            float steer_delta = 0.0f;
            float tmp_duty = servo_motor_duty_middle;

            threshold_value = MAX(0, MIN(255, threshold_value));
            threshold = (unsigned char)threshold_value;
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);

            /* 从近到远逐行试：优先用最近的一条"两条边线都在"的行；
             * 都没有的话退而用最近的一条只有单边的行（靠补线）；
             * 全部失败才是真的丢线。 */
            {
                uint8_t i;
                uint8_t picked = 0xFFU;
                uint8_t single_idx = 0xFFU;
                uint8_t mid_try = (uint8_t)(MT9V03X_W / 2);
                uint8_t picked_mid = (uint8_t)(MT9V03X_W / 2);
                uint8_t picked_state = LANE_NONE;
                uint8_t single_mid = (uint8_t)(MT9V03X_W / 2);
                uint8_t single_state = LANE_NONE;

                for (i = 0U; i < RACE_SAMPLE_ROW_COUNT; i ++)
                {
                    uint8_t row_state = lane_center_at_row(left_line, right_line,
                                                           race_sample_rows[i], MT9V03X_W,
                                                           &mid_try, &race_row_width[i]);
                    if (row_state == LANE_BOTH)
                    {
                        picked = i; picked_mid = mid_try; picked_state = row_state;
                        break;
                    }
                    if ((row_state != LANE_NONE) && (single_idx == 0xFFU))
                    {
                        single_idx = i; single_mid = mid_try; single_state = row_state;
                    }
                }
                if (picked == 0xFFU)
                {
                    picked = single_idx;
                    picked_mid = single_mid;
                    picked_state = single_state;
                }
                race_used_row  = picked;
                race_mid_col   = picked_mid;
                race_lane_state = picked_state;
            }
            sample_row = (race_used_row < RACE_SAMPLE_ROW_COUNT)
                       ? race_sample_rows[race_used_row]
                       : (uint8_t)(MT9V03X_H - 1U);
            mid_err = (float)(MT9V03X_W / 2) - (float)race_mid_col;

            if (race_lane_state != LANE_NONE)
            {
                race_lane_lost_count = 0;
            }
            else if (gray_center > threshold)
            {
                /* 整行全白：说明车还在白带上，只是白带比视野还宽，不是丢线 */
                race_lane_lost_count = 0;
            }
            else if (race_lane_lost_count < 250U)
            {
                /* 中间也是黑的：车已经压到蓝地胶上，真的丢了 */
                race_lane_lost_count ++;
            }
            /* 丢线先保持低速继续走，连续丢够 RACE_LANE_LOST_LIMIT 帧才让 isr 断电机 */
            race_lane_valid = (uint8_t)(race_lane_lost_count < RACE_LANE_LOST_LIMIT);

            if (!race_run_enabled)
            {
                speed_target = 0.0f;
                servo_write(servo_motor_duty_middle);
                race_state = RACE_STATE_STOPPED;
                race_lane_lost_count = 0;   /* 停车时清零，下次按 S2 才能正常起步 */
                race_last_mid_err = mid_err;
            }
            else if (race_lane_valid)
            {
                steer_delta = RACE_STEER_KP * mid_err
                            + RACE_STEER_KD * (mid_err - race_last_mid_err);
                tmp_duty = servo_motor_duty_middle - steer_delta * SERVO_DIR;
                tmp_duty = MAX(tmp_duty, MIN(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
                tmp_duty = MIN(tmp_duty, MAX(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
                servo_write(tmp_duty);

                if (race_lane_state == LANE_NONE)
                {
                    /* 画面里只剩白带（比视野还宽）：低速直行，别加速 */
                    speed_target = RACE_SPEED_TARGET_CURVE;
                    race_state = RACE_STATE_LINE_LOST;
                }
                else
                {
                    speed_target = (mid_err > RACE_CURVE_ERROR_THRESHOLD_PX ||
                                    mid_err < -RACE_CURVE_ERROR_THRESHOLD_PX)
                                 ? RACE_SPEED_TARGET_CURVE
                                 : RACE_SPEED_TARGET_STRAIGHT;
                    race_state = (speed_target == RACE_SPEED_TARGET_CURVE)
                               ? RACE_STATE_CURVE
                               : RACE_STATE_NORMAL;
                }
                race_last_mid_err = mid_err;
            }
            else
            {
                /* 丢线超时：由 race_lane_valid=0 让 isr 断电机 */
                speed_target = 0.0f;
                race_state = RACE_STATE_LINE_LOST;
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

            /* 调试页（每行两个数）：
             *   第1行 L / R      —— 采用那一行扫到的左右边线列号
             *   第2行 err / thr  —— 中线误差 / 二值化阈值
             *   第3行 左灰度 / 中灰度 —— 二值化前，第90行的第5列和第94列的灰度
             *   第4行 行号        —— 实际采用的采样行序号（0~4 对应 90/80/70/60/50，255=全失败） */
            tft180_show_int(0, 0, left_line[sample_row], 3);
            tft180_show_int(64, 0, right_line[sample_row], 3);
            tft180_show_int(0, 18, (int32)mid_err, 3);
            tft180_show_uint(64, 18, threshold, 3);
            tft180_show_uint(0, 36, gray_left, 3);
            tft180_show_uint(64, 36, gray_center, 3);
            tft180_show_int(0, 54, (int32)race_used_row, 3);
            tft180_show_string(0, 110, race_state_text(race_state));

            ++race_frame_id;
            race_send_telemetry(threshold, (int16_t)mid_err, (int16_t)tmp_duty, sample_row);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
    }
}
