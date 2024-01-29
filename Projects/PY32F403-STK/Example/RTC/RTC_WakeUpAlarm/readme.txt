================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示通过RTC闹钟中断每隔1S将MCU从STOP模式下唤醒，每次唤醒会翻转LED，LED翻转
间隔为1s。

Function descriptions:
This example demonstrates waking the MCU from STOP mode every 1 second using an
RTC alarm clock interrupt. Each wake-up will flip the LED, with an LED flip
interval of 1 second.
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
1. 编译并下载程序到MCU；
2. 等待LED灯亮后，按下用户按键，LED灯关闭，进入STOP模式
3. RTC开始计时，每隔1s就会通过闹钟中断唤醒MCU，并且翻转LED 

Example execution steps:
1.Compile and download the program to the MCU.
2.Wait for the LED to turn on, then press the user button to turn off the 
LED and enter the STOP mode.
3.The RTC starts counting, and every 1 second, it will wake up the MCU through 
the alarm interrupt and toggle the LED.
================================================================================
注意事项：

Notes:

================================================================================