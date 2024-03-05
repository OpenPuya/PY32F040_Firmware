================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了定时器的刹车功能，CH1和CH1N互补pwm输出，接收到外部IO口的刹车信号（低
电平）后，PWM信号关闭，由于BDTR.AOE置位，所以刹车信号取消（高电平）后，继续pwm输
出，此样例实现了死区功能。
CH1   ->  PA8
CH1N  ->  PA7
刹车输入 -> PA6
通过调整OCxE,CCxP,OISx,CCxNE,CCxNP,OISxN的配置，可实现刹车功能的各种应用

Function descriptions:
This sample demonstrates brake function of the timer,the CH1 and CH1N 
complementary pwm outputs.After receiving the brake signal (low level)from the 
external IO port, the PWM signal is turned off. Because BDTR.AOE is set, the pwm
output continues after the brake signal is cancelled (high level). This example 
realizes the dead zone function
CH1 -> PA8
CH1N -> PA7
Brake input -> PA6
By adjusting the OCxE, CCxP, OISx, CCxNE, CCxNP, OISxN configuration, which can 
realize the brake function of a variety of applications
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
1. 下载并运行程序
2. 通过逻辑分析仪捕捉上述两个通道的电平，判断pwm输出信号是否正确
3. PA6输入低电平，判断刹车信号是否生效

Example execution steps:
1. compile and download the program to MCU and run it;
2. Capture the levels of the above two channels through the logic analyzer to 
determine whether the pwm output signal is correct
3. Input low level into PA6 to determine whether the brake signal is effective
================================================================================
注意事项：
死区时间计算:
DTG[7:5]=0xx => DT=DTG[7:0] × Tdtg， Tdtg = TDTS；
DTG[7:5]=10x => DT=(64+DTG[5:0]) × Tdtg， Tdtg = 2 × TDTS；
DTG[7:5]=110 => DT=(32+DTG[4:0]) × Tdtg， Tdtg = 8 × TDTS；
DTG[7:5]=111 => DT=(32+DTG[4:0]) × Tdtg， Tdtg = 16 × TDTS；
例：若 TDTS = 125ns(8MHZ)，可能的死区时间为：
0 到 15875ns，若步长时间为 125ns；
16us 到 31750ns，若步长时间为 250ns；
32us 到 63us，若步长时间为 1us；
64us 到 126us，若步长时间为 2us；

Notes:
Dead time calculation
DTG[7:5]=0xx => DT=DTG[7:0] × Tdtg， Tdtg = TDTS；
DTG[7:5]=10x => DT=(64+DTG[5:0]) × Tdtg， Tdtg = 2 × TDTS；
DTG[7:5]=110 => DT=(32+DTG[4:0]) × Tdtg， Tdtg = 8 × TDTS；
DTG[7:5]=111 => DT=(32+DTG[4:0]) × Tdtg， Tdtg = 16 × TDTS；
Example if TDTS=125 ns (8 MHz), dead-time possible values are:
0 to 15875 ns by 125 ns steps,
16 us to 31750 ns by 250 ns steps,
32 us to 63 us by 1 us steps,
64 us to 126 us by 2 us steps
================================================================================