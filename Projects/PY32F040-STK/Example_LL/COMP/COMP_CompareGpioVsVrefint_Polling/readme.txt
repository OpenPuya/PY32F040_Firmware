================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了比较器的轮询功能，当比较器的正端电压大于Vrefint时，LED灯亮，小于
Vrefint电压时，LED灯灭。

Function descriptions:
This example demonstrates the polling function of the comparator. When the
positive terminal voltage of the comparator is greater than Vrefit, the LED
light will turn on, and when the voltage is less than Vrefit, the LED light
will turn off.
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
2. PA0输入大于1.2V电压，LED亮；
3. PA0输入小于1.2V电压，LED灭；

Example execution steps:
1. Compile and download the program and run it
2. When the input voltage of PA0 is greater than 1.2V, the LED is on;
3. If the input voltage of PA0 is less than 1.2V, the LED will turn off.
================================================================================
注意事项：

Notes:

================================================================================