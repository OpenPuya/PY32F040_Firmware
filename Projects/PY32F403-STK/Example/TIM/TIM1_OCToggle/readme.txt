================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1比较模式下的OC翻转输出功能，使能CH1(PA08),CH2(PA09),CH3(PA10),
CH4(PA11)四个通道的输出功能，并且当计数器TIMx_CNT与TIMx_CCRx匹配时输出信号翻转
，频率为100KHz。

Function descriptions:
This sample demonstrates the OC toggle output function in TIM1 comparison mode, 
enabling CH1(PA08),CH2(PA09),CH3(PA10),CH4(PA11) four channel output function, 
then the output signal toggle when the counter TIMx_CNT matches TIMx_CCRx.
The frequency is 100KHz
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
1. 编译下载并运行程序；
2. 用示波器监测PA08/PA09/PA10/PA11引脚，可观察到与程序中配置相匹配的PWM波形；

Example execution steps:
1. compile and download the program to MCU and run it;
2. By monitoring PA08/PA09/PA10/PA11 pin with oscilloscope, the PWM waveform 
matching the configuration in the program can be observed;
================================================================================
注意事项：

Notes:

================================================================================