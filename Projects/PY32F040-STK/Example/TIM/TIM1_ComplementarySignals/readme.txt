================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例实现了定时器的互补输出功能，三组互补共六路pwm输出，此样例没有实现死区功能
CH1   ->  PA8
CH1N  ->  PA7
CH2   ->  PA9
CH2N  ->  PB0
CH3   ->  PA10
CH3N  ->  PB1

Function descriptions:
This sample demonstrates complementary output function of the timer,Three sets 
of complementary outputs total six pwm outputs, this example does not implement 
the dead zone function
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
2. 通过逻辑分析仪捕捉上述六通道的电平，判断pwm输出信号是否正确

Example execution steps:
1. compile and download the program to MCU and run it;
2. Capture the level of the above six channels through the logic analyzer to 
determine whether the pwm output signal is correct
================================================================================
注意事项：

Notes:

================================================================================