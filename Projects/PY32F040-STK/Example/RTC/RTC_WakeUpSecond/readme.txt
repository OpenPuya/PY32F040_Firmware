================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过RTC的秒中断唤醒MCU的功能。下载程序并运行后，LED灯处于常亮状态；
按下用户按键后，LED灯处于常暗状态，且MCU进入STOP模式；RTC秒中断唤醒MCU后，LED灯
处于闪烁状态。

Function descriptions:
This sample demonstrates waking up the MCU using RTC second interrupt. After 
downloading and running the program, the LED is continuously on. Pressing the 
user button turns off the LED and puts the MCU into STOP mode. When the RTC 
second interrupt wakes up the MCU, the LED starts blinking.
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
1. 编译并下载程序到MCU，并运行。
2. 初始状态下，LED灯处于常亮状态。
3. 按下用户按键后，LED灯处于常暗状态，MCU进入STOP模式。
4. 当RTC秒中断唤醒MCU时，LED灯开始闪烁。

Example execution steps:
1. Compile and download the program to the MCU, and run it.
2. In the initial state, the LED is continuously on.
3. Press the user button to turn off the LED and put the MCU into STOP mode.
4. When the RTC second interrupt wakes up the MCU, the LED starts blinking.
================================================================================
注意事项：

Notes:
================================================================================
