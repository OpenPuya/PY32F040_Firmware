================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1的外部时钟模式1功能，选择ETR(PA12)引脚作为外部时钟输入源，并使能
更新中断，在中断中翻转LED灯

Function descriptions:
This sample demonstrates external clock mode 1 function of the TIM1.Select the 
ETR(PA12) pin as the external clock input source and enable the update interrupt 
to flip the LED light in the interrupt.
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
1. 编译下载程序到MCU，并运行；
2. PA12引脚，输入外部时钟；
3. 可观察到MCU从PA12引脚上每检测到800个脉冲，会让LED灯翻转一次；

Example execution steps:
1. compile and download the program to MCU and run it;
2. Input external clock into PA12 pin;
3. It can be observed that every 800 pulses detected by the MCU from the PA12 
pin, the LED light will be turned over once;
================================================================================
注意事项：

Notes:

================================================================================