# smartcar-camera-group

这是智能车摄像头组的初始例程代码，目前仅用于实现低速直线行驶。

## 工程说明

- 主控平台：RT1064DVL6A
- 开发环境：Keil MDK 5.33 或 IAR 8.32.4
- Keil 工程：`demo/mdk/rt1064.uvprojx`
- 主要用户代码：`demo/user/`
- 底层库：`libraries/`

## 使用前注意

烧录前请根据实际接线检查 `demo/user/inc/main.h` 中的电机、编码器、舵机和摄像头引脚定义。首次上电测试时请将车轮架空，并使用较低目标速度。

