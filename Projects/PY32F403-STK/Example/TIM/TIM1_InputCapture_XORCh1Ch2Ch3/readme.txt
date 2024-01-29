================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1输入捕获功能，PA8或PA9或PA10输入时钟信号，TIM1捕获成功后，
会进入捕获中断，每进一次中断，翻转一次LED。

Function descriptions:
This sample demonstrates the input capture function of TIM1, input clock signal 
into PA8 or PA9 or PA10 will generate the capture interrupt after TIM1 capture 
successfully.Toggle the LED once per interruption
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
2. PA8和PA10引脚不输入时钟信号的情况下，PA9引脚输入时钟信号，LED会翻转
3. PA8和PA9引脚不输入时钟信号的情况下，PA10引脚输入时钟信号，LED会翻转
4. PA9和PA10引脚不输入时钟信号的情况下，PA8引脚输入时钟信号，LED会翻转

Example execution steps:
1. compile and download the program to MCU and run it;
2. When PA8 and PA10 pins do not input the clock signal, input the clock signal 
into PA9 pins, and the LED will toggle
3. When PA8 and PA9 pins do not input the clock signal, input the clock signal 
into PA10 pins, and the LED will toggle
4. When PA9 and PA10 pins do not input clock signals, input the clock signal 
into PA8 pins, and the LED will toggle
================================================================================
注意事项：

Notes:

================================================================================