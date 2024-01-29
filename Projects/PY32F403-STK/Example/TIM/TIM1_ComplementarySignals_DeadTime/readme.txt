================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了定时器的刹车功能，CH1和CH1N互补pwm输出，接收到外部IO口的刹车信号（低
电平）后，PWM信号关闭，由于BDTR.AOE置位，所以刹车信号取消（高电平）后，继续pwm输
出，此样例实现了死区功能。

Function descriptions:
This example realizes the braking function of the timer, CH1 and CH1N complement 
pwm output, after receiving the brake signal (low level) of the external IO port
the PWM signal is turned off.Because the BDTR.AOE is set, so after the brake 
signal is cancelled (high level), and the pwm output continues. The dead time is
inserted in the complementary output of CH1 and CH1N
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
2. 通过逻辑分析仪捕捉上述两个通道的电平，判断pwm输出信号是否正确
3. PB12输入低电平，判断刹车信号是否生效

Example execution steps:
1. compile and download the program to MCU and run it;
2. Capture the level of the above six channels through the logic analyzer to 
determine whether the pwm output signal is correct
3. Low level input into PB12 ,determine whether the brake signal is effective
================================================================================
注意事项：
1. 引脚
CH1   ->  PA8
CH1N  ->  PB13
刹车输入 -> PB12
2. 通过调整OCxE,CCxP,OISx,CCxNE,CCxNP,OISxN的配置，可实现刹车功能的各种应用

Notes:
1. Pins
CH1   ->  PA8
CH1N  ->  PB13
BKIN -> PB12
2. By adjusting the OCxE CCxP, OISx CCxNE, CCxNP, OISxN configuration, which can
realize the brake function of a variety of applications
================================================================================