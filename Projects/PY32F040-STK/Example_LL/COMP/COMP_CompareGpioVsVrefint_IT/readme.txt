================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了比较器的中断功能，在中断中翻转LED。

Function descriptions:
This example demonstrates the interrupt function of the comparator, flipping
the LED during the interrupt.
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
2. PA0的电压值大于1.2V时，LED灯亮，小于1.2V时，LED灭.

Example execution steps:
1. Compile and download the program and run it
2. When the voltage value of PA0 is greater than 1.2V, the LED lights up, and
when it is less than 1.2V, the LED goes out
================================================================================
注意事项：

Notes:

================================================================================