# smartcar-camera-group

这是从“镜头组例程”保留出来的正式调试工程。原例程不再修改；本目录使用 Git 保存每一个可回退基线。

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
- 主板 S2 短按一次进入运行，再短按一次停车。
- 当前控制行同时识别不到左右边线时显示 `LOST` 并停止电机。
- 主板 S1 短按后停车，并通过 115200 波特率调试串口输出一帧原始 P5 PGM；采集结束后仍保持停车，必须再按 S2 才能运行。
- 屏幕状态：`STOP` 停车、`NORM` 普通巡线、`CURV` 弯道降速、`LOST` 丢线、`CAPT` 正在输出图像。

S1/S2 使用逐飞库原有的按键定义，本阶段没有修改 `main.h` 中任何硬件引脚、舵机中值或机械极限。

## 参数与遥测

`demo/user/inc/race_autotune_generated.h` 由 `../赛道自优化工具` 生成，不要手工修改。默认参数保持低速：直道目标 30、弯道目标 24；串口遥测默认关闭。

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

`expected-center` 是固定控制行（距图像底部 30 行）上人工判断的正确中线横坐标。正式让工具自动改代码前，必须用真实帧替换演示数据并通过回放。

