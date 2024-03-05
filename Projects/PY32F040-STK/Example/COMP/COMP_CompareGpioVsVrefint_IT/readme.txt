================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了COMP比较器中断功能，PA00作为比较器正端输入，VREFINT作为比较器负端输
入，当PA0的电压大于Vref电压时，LED灯亮，小于Vref电压时,LED灯灭。

Function descriptions:
This example demonstrates the interrupt function of the COMP comparator, with
PA00 as the positive input and VREFINT as the negative input. When the voltage
of PA0 is greater than Vref voltage, the LED lights up, and when it is less
than Vref voltage, the LED lights out.
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
2. 配置PA0输入1.1V电压，LED灯灭
3. 配置PA0输入1.3V电压, LED灯亮

Example execution steps:
1.Compile and download the program to MCU and run it;
2.Configure PA0 input voltage of 1.1V, LED light off
3.Configure PA0 input 1.3V voltage, LED light on
================================================================================
注意事项：

Notes:

================================================================================