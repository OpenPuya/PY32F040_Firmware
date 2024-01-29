================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1的外部时钟模式1功能，选择TI1FD(PA8)引脚作为外部时钟输入源，并使能
更新中断，在中断中翻转LED灯。

Function descriptions:
This sample demonstrates the external clock mode 1 function of TIM1, selects the 
TI1FD(PA8) pin as the external clock input source, and enables the update 
interrupt and toggle the LED light in the interrupt
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
2. PA8引脚，输入外部时钟；
3. 可观察到MCU从PA8引脚上输入时钟信号，CNT计数器计到800时，产生更新事件，LED翻转
一次。

Example execution steps:
1. compile and download the program to MCU and run it;
2. Input external clock into PA8 pin;
3. It can be observed that the MCU receives the input clock signal from the PA8 
pin, and when the CNT counter reaches 800, an update event is generated, and the 
LED toggle once
================================================================================
注意事项：

Notes:

================================================================================