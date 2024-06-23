================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了定时器的基本计数功能，以及演示了ARR自动重载功能，样例在定时器重载中断
中翻转LED灯修改main.c中的
配置TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;使能自动重载
功能，新的ARR值在第四次进中断时生效，配置TimHandle.Init.AutoReloadPreload = 
TIM_AUTORELOAD_PRELOAD_DISABLE;禁止自动重载功能，新的ARR值在第三次进中断时生效,
生效后，LED灯以2.5HZ的频率翻转

Function descriptions:
This sample demonstrates base count function of the timer,and show ARR register
autoreload function.Example toggle LED in timer update interrupt.
Modify in main.c.
Set TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE to enable
autoreload,and new ARR value will takes effect on the fourth interrupt generate.
Set TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE to disable
autoreload,and new ARR value will takes effect on the third interrupt generate.
After taking effect, the LED lights blinked at a frequency of 2.5HZ.
================================================================================
测试环境：
测试用板：PY32F040_STK
MDK版本： 5.28
IAR版本： 9.20
GCC 版本：GNU Arm Embedded Toolchain 10.3-2021.10

Test environment:
Test board: PY32F040_STK
MDK Version: 5.28
IAR Version: 9.20
GCC Version: GNU Arm Embedded Toolchain 10.3-2021.10
================================================================================
使用步骤：
1. 编译下载程序并运行；
2. LED灯闪烁，用示波器观测PB2引脚
3. PB2在第三次翻转时，周期变为2.5Hz

Example execution steps:
1. compile and download the program to MCU and run it;
2. LED light blinked, observe PB2 pin with oscilloscope
3. When PB2 is flipped for the third time, the period becomes 2.5Hz
================================================================================
注意事项：

Notes:

================================================================================