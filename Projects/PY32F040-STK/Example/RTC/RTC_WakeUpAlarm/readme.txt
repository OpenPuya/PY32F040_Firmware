================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示通过RTC闹钟中断每隔1秒将MCU从STOP模式下唤醒，并且每次唤醒会翻转LED，LED翻转
间隔为1秒。

Function descriptions:
This sample demonstrates waking up the MCU from STOP mode every 1 second using 
RTC alarm interrupt. Each time the MCU wakes up, the LED will toggle its state. 
The LED toggling interval is 1 second.
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
1. 编译并下载程序到MCU。
2. 等待LED灯亮起后，按下用户按键，LED灯关闭，MCU进入STOP模式。
3. RTC开始计时，每隔1秒通过闹钟中断唤醒MCU，并且翻转LED的状态。

Example execution steps:
1. Compile and download the program to the MCU.
2. Wait for the LED to turn on, then press the user button to turn off the LED 
   and put the MCU into STOP mode.
3. The RTC starts counting, and every 1 second, the MCU is woken up by the RTC 
   alarm interrupt, and the LED state is toggled.
================================================================================
注意事项：

Notes:
================================================================================
