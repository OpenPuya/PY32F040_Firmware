================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了WWDG的提前唤醒中断功能，看门狗计数器向下计数到0x40时产生中断，中断中
喂狗，可以确保看门狗不会复位。

Function descriptions:
This example demonstrates early wake up interrupt function of the WWDG.When the 
watchdog counter counts down to 0x40 will generates an interrupt.Refresh the
WWDG in interrupt to ensure that the WWDG does not reset.
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
2. 程序在看门狗提前唤醒中断中喂狗，LED以1hz在闪烁。

Example execution steps:
1. compile and download the program to MCU and run it.
2. The program refresh in the wwdg early wake-up interrupt, and the LED blinks 
at 1hz.
================================================================================
注意事项：

Notes：

================================================================================