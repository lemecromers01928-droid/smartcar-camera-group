# 中期检查初版（摄像头组）

这是用于中期检查的正式工程，从仓库里的 [`初始例程/`](../初始例程/) 迭代而来。初始例程不再改动；本目录每跑通一次就提交一个可回退基线。

对应中期检查时间：2026-10-24 ~ 2026-10-25。

## 当前实车表现

2026-10-19 实车验证：

- **能连续跑完中期检查全程，用时约 30 s**（直线 1 m 约 2~3 s，即 0.3~0.5 m/s）
- 直道能稳定居中；常规弯道不压线；手推偏测试通过（人为把车头推偏 1~2 cm，车能自己修回来）
- 待优化：**急 S 弯车头会压线**。转向行程 ±21° 对 R = 0.5 m 的弯刚好够，再加上控制行前瞻偏近，属于控制品质问题，不是逻辑错误
- 下一步：先连跑 10 次统计出界/中断次数，再决定要不要调参。当前这一版是能过检的基线，不要随意改

## 工程说明

- 主控平台：RT1064DVL6A
- 开发环境：Keil MDK 5.33 或 IAR 8.32.4
- Keil 工程：`demo/mdk/rt1064.uvprojx`
- 主要用户代码：`demo/user/`
- 底层库：`libraries/`

## 使用前注意

烧录前请根据实际接线检查 `demo/user/inc/main.h` 中的电机、编码器、舵机和摄像头引脚定义。首次上电测试时请将车轮架空，并使用较低目标速度。

## 当前安全逻辑

- 上电后默认 `STOP`，两路电机 PWM 为 0，舵机保持已标定中值。
- 主板物理 S2 短按一次进入运行，再短按一次停车。实机验证该按键对应逐飞库的 `KEY_1/C15`。
- 控制行不固定：从近到远依次试第 90/80/70/60/50 行，优先用最近的一条“两条边线都在画面里”的行；只有一条边线时用参考带宽补出另一条（`lane_center_at_row`）。
- 只有两条边线都找不到、并且画面中心也是黑的（车压到蓝地胶上）才累计丢线；连续丢 40 帧（约 0.4~0.8 s）才断电机。整屏全白说明车还在白带上，继续保持低速直行。
- 长按 C15 进入舵机自检（屏幕显示 `SVO`）：进场会左右各摆 15° 一次；短按 C15 每格 2°、短按 C14 反向，用来确认转向方向和找机械极限。
- 主板物理 S1 短按后停车，并通过 115200 波特率调试串口输出一帧原始 P5 PGM；采集结束后仍保持停车，必须再按 S2 才能运行。
- 屏幕状态：`STOP` 停车、`NORM` 普通巡线、`CURV` 弯道降速、`LOST` 丢线、`CAPT` 正在输出图像。

按键使用逐飞库原有的 C14/C15 引脚（`KEY_2` / `KEY_1`），仅根据实机丝印交换了操作功能。

**舵机极限已按实车标定**：`SERVO_MOTOR_L_MAX = 112`、`SERVO_MOTOR_R_MAX = 70`（中值 91），另有 `SERVO_MOTOR_CENTER_TRIM` 用于微调中值。C30 和 C31 两个舵机口同时输出——只写一个口时，如果舵机线插在另一个口，表现就是“舵机完全不动”，这个坑实际踩过。

## 参数与遥测

`demo/user/inc/race_autotune_generated.h` 由 `../赛道自优化工具` 生成，不要手工修改。默认参数保持低速：直道目标 30、弯道目标 24；串口遥测默认关闭。

速度环是“前馈 + 位置式 PI + 斜率限制”，不是纯 PID：前馈 `speed_feedforward = 2500`（`pid.c`）、`RACE_SPEED_KP = 60`、每 10 ms 占空比最多变化 300（`main.h` 的 `RACE_SPEED_SLEW`）。只靠比例项时静止占空比最多 `Kp × 目标转速`，车根本推不动——这是之前“按了 S2 电机转得很慢、车走不动”的根因。

需要现场遥测时，把 `../赛道自优化工具/race_tuner.json` 的 `RACE_TELEMETRY_ENABLE` 改为 `1`，然后在工具目录执行：

```powershell
python -m race_tuner init
```

重新编译烧录后，遥测格式为：

```text
@RACE,frame=...,state=...,thr=...,err=...,servo=...,target=...,enc_l=...,enc_r=...,left=...,right=...,valid=...
```

## 采集一帧原始灰度图

先确认串口号，例如：

```powershell
[IO.Ports.SerialPort]::GetPortNames()
```

在本工程目录运行采集器，再短按 S1：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\capture_pgm.ps1 -Port COM5
```

文件保存到 `captures/`。采集时车会停车，采集后不会自动恢复运行。

## 记录实车遥测

先按上述方法启用遥测并重新编译烧录，然后运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\capture_telemetry.ps1 `
  -Port COM5 -Seconds 60
```

日志保存到 `logs/`，可交给赛道自优化工具计算最大中线误差、舵机振荡和速度 RMSE。

## 离线回放

在 `../赛道自优化工具` 中导入人工标注后的真实帧：

```powershell
python -m race_tuner import-frame '..\摄像头组工程\captures\frame-时间戳.pgm' `
  --expected-center 94 --label straight --tolerance 5
python -m race_tuner replay
```

`expected-center` 是人工判断的正确中线横坐标。注意：固件的控制行已经改成自适应（90/80/70/60/50 逐行尝试），而回放工具仍按固定的“距图像底部 30 行”计算，两者不完全等价；正式用回放做参数门禁前需要先把这一点对齐。正式让工具自动改代码前，必须用真实帧替换演示数据并通过回放。

## 相对初始例程的主要改动

| 文件 | 改动 |
| --- | --- |
| `demo/user/src/isr.c` | 速度环改为“前馈 + 位置式 PI + 斜率限制”；停车时清积分；按键扫描放进 10 ms 定时器 |
| `demo/user/src/pid.c` | 新增 `speed_feedforward`；修正 `PidLocCtrl` 的积分限幅（原来限的是临时变量，等于没限） |
| `demo/user/src/img_process.c` | 修正直方图清零长度（`memset` 只清了 128 个 short）；修正边线数组未初始化；新增单边补线 `lane_center_at_row` |
| `demo/user/src/main.c` | 启停按键、舵机自检、自适应采样行、丢线超时、调试屏 |
| `demo/user/inc/main.h` | 舵机极限按实车标定 112/70；新增 `RACE_SPEED_SLEW`、`RACE_LANE_LOST_LIMIT`、`SERVO_MOTOR_CENTER_TRIM`、`SERVO_MOTOR_PWM_ALT` |
| `demo/user/inc/race_autotune_generated.h` | 参数外置（阈值偏置、转向 PD、直弯速度、速度环） |
| 新增文件 | `scripts/capture_pgm.ps1`、`scripts/capture_telemetry.ps1`、`demo/host_tests/replay_runner.c` |

