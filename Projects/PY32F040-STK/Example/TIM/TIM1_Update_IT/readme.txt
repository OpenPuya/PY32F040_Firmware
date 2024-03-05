================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1中基本计数功能，并使能了更新中断，每次重装ARR值时会产生一次
更新中断，并在中断中翻转LED灯，LED灯会以5Hz的频率进行翻转。

Function descriptions:
This sample demonstrates basic count function of the TIM1 and enable update 
interrupt.Each time an update interrupt is generated, the ARR value is reloaded 
and the LED light is toggled in the interrupt.The LED light is toggled at a 
frequency of 5Hz.
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
2. LED灯闪烁

Example execution steps:
1. compile and download the program to MCU and run it;
2. The LED light blinks.
================================================================================
注意事项：

Notes:

================================================================================