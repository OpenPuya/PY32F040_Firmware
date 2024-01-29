================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了TIM1和TIM3级联成32位计数器，TIM3做主机，TIM3的计数溢出信号作为TIM1的
输入时钟，通过配置TIM1和TIM3的重载寄存器值，（在TIM1中断回调函数中）实现LED灯以
0.5Hz频率闪烁。

Function descriptions:
This example realizes the cascade of TIM1 and TIM3 into a 32-bit counter, with 
TIM3 as the host.The count overflow signal of TIM3 acts as the input clock of 
TIM1.By configuring the reloaded register values of TIM1 and TIM3, the LED 
is toggled at 0.5Hz (in the TIM1 interrupt callback function).
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
2. LED灯以0.5Hz频率闪烁

Example execution steps:
1. compile and download the program to MCU and run it;
2. The LED blink at 0.5Hz
================================================================================
注意事项：
默认主频为8M
此例程计算方式为TIM3_ARR*TIM3_PSC*TIM1_ARR*TIM1_PSC/时钟
=800*100*100*1/8000000=1Hz

Notes:
Default system clock is 8M
the calculation method of example : TIM3_ARR*TIM3_PSC*TIM1_ARR*TIM1_PSC/ clock
=800*100*100*1/8000000=1Hz
================================================================================