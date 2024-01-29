================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了standby模式下，通过wakeuppin唤醒功能。

Function descriptions:
This sample demonstrates the wake-up feature using the wakeup pin in standby 
mode. When the program starts running, the LED will be on. Press the button, 
and the LED will turn off, entering standby mode. When a rising edge is detected 
on PA02, the program will exit standby mode. The LED will blink for 5 seconds 
and then be turned on again.

================================================================================
测试环境：
测试用板：PY32F403_STK
MDK版本：5.28
IAR版本：9.20

Test environment:
Test board: PY32F403_STK
MDK Version: 5.28
IAR Version: 9.20

================================================================================
使用步骤：
1. 编译下载程序到MCU，断电并重新上电；
2. LED灯亮，按下按键，LED灯灭，进入standby模式；
3. 在PA02上产生一个上升沿，程序退出standby模式，LED闪烁5秒，然后被重新点亮。

Example execution steps:
1. Compile and download the program to the MCU, disconnect power, and then 
   power up again;
2. The LED will be on initially. Press the button, and the LED will turn off, 
   entering standby mode;
3. Generate a rising edge on PA02, and the program will exit standby mode. 
   The LED will blink for 5 seconds and then be turned on again.

================================================================================
注意事项：

Notes:

================================================================================
