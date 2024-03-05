================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了COMP比较器唤醒功能，PA0作为比较器正端输入，VREFINT作为比较器负端输
入，上完电LED灯会常亮，用户点击按键，LED灯灭，进入stop模式，通过调整PA0上的输入
电压，产生中断唤醒stop模式。

Function descriptions:
This example demonstrates the COMP comparator wake-up function, with PA0 as the
positive input and VREFINT as the negative input. After power on, the LED light
will remain on. When the user clicks the button, the LED light will go out and
enter stop mode. By adjusting the input voltage on PA0, an interrupt wake-up
stop mode is generated.
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
1. PA0上输入小于1.1V的电压;
2. 编译下载程序到MCU，并运行，发现LED灯常亮；
3. 按下按键，LED灯熄灭，进入stop模式；
4. 在PA0上输入大于1.3V的电压，产生中断唤醒；
5. 接下来LED每隔200ms进行翻转，程序正常运行。

Example execution steps:
1. Input voltage less than 1.1V on PA0;
2. Compile and download the program to MCU, and run it. It is found that the
LED light is constantly on;
3. Press the button to turn off the LED light and enter stop mode;
4. Input a voltage greater than 1.3V on PA0 to generate interrupt wake-up;
5. Next, the LED flips every 200ms and the program runs normally.
================================================================================
注意事项：

Notes:

================================================================================