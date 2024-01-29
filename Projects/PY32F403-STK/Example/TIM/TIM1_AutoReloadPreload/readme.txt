================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了定时器的基本计数功能，以及演示了ARR自动重载功能，样例在定时器重载中断
中翻转LED灯。

Function descriptions:
This sample demonstrates base count function of the timer,and show ARR register
autoreload function.Example toggle LED in timer update interrupt.
================================================================================
测试环境：
测试用板：PY32F403_STK
MDK版本： 5.28
IAR版本： 9.20

Test environment:
Test board: PY32F403_STK
MDK Version: 5.28
IAR Version: 9.20
================================================================================
使用步骤：
1. 编译下载程序并运行；
2. LED灯闪烁，用示波器观测PA1引脚
3. PA1在第三次翻转时，周期变为2.5Hz

Example execution steps:
1. compile and download the program to MCU and run it;
2. LED light blinked, observe PA1 pin with oscilloscope
3. When PA1 is flipped for the third time, the period becomes 2.5Hz
================================================================================
注意事项：
修改main.c中的第57行
配置TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;使能自动重载
功能，新的ARR值在第四次进中断时生效；
配置TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;禁止自动重
载功能，新的ARR值在第三次进中断时生效,生效后，LED灯以2.5HZ的频率翻转。

Notes:
Modify line 56 in main.c.
Set TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE to enable
autoreload,and new ARR value will takes effect on the fourth interrupt generate.
Set TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE to disable
autoreload,and new ARR value will takes effect on the third interrupt generate.
After taking effect, the LED lights blinked at a frequency of 2.5HZ.
================================================================================