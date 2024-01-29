================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1(PA8)输入捕获功能，PA8输入时钟信号，TIM1捕获成功后，会进入
捕获中断，每进一次中断，翻转一次LED。

Function descriptions:
This sample demonstrates the input capture function of TIM1(PA8), PA8 input 
clock signal, when TIM1 capture success, will enter the capture interrupt,and
toggle the LED in the interrupt
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
1. 下载并运行程序
2. PA8引脚不输入时钟信号的情况下，LED不翻转
3. PA8输入时钟信号，TIM1捕获成功后，LED会翻转

Example execution steps:
1. compile and download the program to MCU and run it;
2. The LED does not toggle when the PA8 pin does not input the clock signal
3. PA8 input clock signal, TIM1 capture success, LED will toggle
================================================================================
注意事项：

Notes:

================================================================================