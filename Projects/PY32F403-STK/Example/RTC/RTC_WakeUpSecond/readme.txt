================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过RTC的秒中断唤醒MCU的功能。下载程序并运行后，LED灯处于常亮状态；
按下用户按键后，LED灯处于常暗状态，且MCU进入STOP模式；RTC秒中断唤醒MCU后，LED灯
处于闪烁状态。

Function descriptions:
This sample demonstrates the functionality of waking up the MCU using the RTC 
second interrupt. After downloading and running the program, the LED will be 
constantly on. Pressing the user button will turn off the LED and put the MCU 
into the STOP mode. When the RTC second interrupt wakes up the MCU, the LED 
will blink.
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
1. 编译并下载程序到MCU，并运行；
2. 小灯处于常亮状态，按下用户按键，LED灯处于常暗状态，且MCU进入STOP模式
3. 秒中断唤醒MCU后，小灯处于闪烁状态

Example execution steps:
1.Compile and download the program to the MCU and run it.
2.The LED will be constantly on. Press the user button to turn off the LED and 
put the MCU into the STOP mode.
3.After the RTC second interrupt wakes up the MCU, the LED will blink.
================================================================================
注意事项：

Notes:
================================================================================