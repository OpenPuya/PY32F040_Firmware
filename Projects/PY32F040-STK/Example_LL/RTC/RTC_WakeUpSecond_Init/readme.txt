================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示通过RTC秒中断从STOP模式下唤醒，唤醒后，小灯处于闪烁状态；否则处于熄灭
状态。

Function descriptions:
This sample demonstrates waking up the MCU from STOP mode using RTC second 
interrupt. After waking up, the LED will either blink or remain off.
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
1. 编译并下载程序到MCU；
2. 等待LED灯亮后，按下用户按键，LED灯关闭，进入STOP模式；
3. 通过秒中断唤醒MCU后，LED灯闪烁。

Example execution steps:
1. Compile and download the program to the MCU.
2. Wait for the LED to turn on, press the user button to turn off the LED and 
   enter STOP mode.
3. After the MCU is awakened by the second interrupt, the LED will blink.
================================================================================
注意事项：

Notes:
================================================================================
