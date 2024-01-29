================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了sleep模式下，通过GPIO事件唤醒功能。

Function descriptions:
This sample demonstrates the GPIO wake-up feature in sleep mode.
When the program starts running, the LED will be on. Press the button, and the 
LED will turn off, entering sleep mode. When a falling edge is detected on PA06, 
the program will exit sleep mode. The LED will toggle at intervals of 500ms.

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
1. 编译下载程序到MCU，并运行；
2. LED灯亮，按下按键，LED灯灭，进入sleep模式；
3. 在PA06上产生一个下降沿，程序退出sleep模式；
4. LED以500ms的间隔进行翻转。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. The LED will be on initially. Press the button, and the LED will turn off, 
   entering sleep mode;
3. Generate a falling edge on PA06, and the program will exit sleep mode;
4. The LED will toggle at intervals of 500ms.

================================================================================
注意事项：

Notes:

================================================================================
