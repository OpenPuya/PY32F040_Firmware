================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1的单脉冲模式，CH2(PA09)引脚上的上升沿，触发计数器开始计数，当计数
值与CCR1匹配时，CH1(PA08)输出高电平，直到计数器溢出，CH1再次输出低电平，计数器溢
出后，定时器停止工作，
本例程脉冲宽度计算 (TIM1_ARR-TI1_CCR1)/CLK=（65535-16383）/32000000=1.536ms

Function descriptions:
This sample demonstrates the one pulse mode of TIM1.The rising edge on the 
CH2(PA09) pin triggers the counter to start counting. when the count value 
matches CCR1,CH1(PA08) outputs a high level. When the counter overflows ,CH1 
outputs the low level again. After the counter overflows, the timer stops 
working.
This example pulse width calculation 
(TIM1_ARR-TI1_CCR1)/CLK= (65,535-16383)/ 32,000,000 =1.536ms
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
1. 编译下载程序到MCU，并运行；
2. 在PA09上产生一个上升沿；
3. 通过示波器监测PA09和PA08引脚，可监测到波形;

Example execution steps:
1. compile and download the program to MCU and run it;
2. Create a rising edge on PA09;
3. Through the oscilloscope monitoring PA09 and PA08 pins, waveform can be 
observed;
================================================================================
注意事项：
输出效果见下图：
                                ____
                                |   |
   CH2 _________________________|   |_________________________________________
   
                                              ___________________________
                                             |                           |
   CH1 ______________________________________|                           |____
                                <---Delay----><------Pulse--------------->

Notes:
The output effect is shown below:
                                ____
                                |   |
   CH2 _________________________|   |_________________________________________
   
                                              ___________________________
                                             |                           |
   CH1 ______________________________________|                           |____
                                <---Delay----><------Pulse--------------->
================================================================================