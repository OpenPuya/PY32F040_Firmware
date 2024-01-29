================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了TIM1中的编码器计数功能，TI1(PA8)和TI2(PA9)作为编码器输入引脚，通过
CNT寄存器可观察到计数器变化，通过uwDirection变量可观察到计数器的计数方向，通过
打印数据也可观察计数方向和CNT寄存器计数值，打印数据Direction = 0为向上计数，
Direction = 1为向下计数。

Function descriptions:
This sample demonstrates encoder count function of the TIM1,TI1(PA8) and
TI2(PA9) configured as encoder input pins.The change of the counter can be
observed through the CNT register, and the counting direction of the counter can
be observed through the uwDirection variable.The counting Direction and CNT
register can also be observed by printing data. The printed data Direction = 0
indicates CounterMode:Up, and direction = 1 indicates CounterMode:down.
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
1. 编译下载程序到MCU，并通过debug模式运行；
2. 在PA08和PA09上输入交替脉冲（PA08上升沿在前，PA09上升沿在后，具体请参考用户手册）；
3. 可观察到CNT寄存器的值不断累加；
4. 在PA08和PA09上输入交替脉冲（PA09上升沿在前，PA08上升沿在后，具体请参考用户手册）；
5. 可观察到CNT寄存器的值不断减小；

Example execution steps:
1. compile and download the program to MCU and run it through debug mode;
2. Input alternating pulses on PA08 and PA09 (PA08 rising edge in front, PA09 
rising edge in back, please refer to the user manual for details);
3. It can be observed that the value of the CNT register increase continuously.
4. Input alternating pulses on PA08 and PA09 (PA09 rising edge in front, PA08 
rising edge in back, please refer to the user manual for details);
5. The value of the CNT register can be observed to decrease continuously;
================================================================================
注意事项：

Notes:

================================================================================