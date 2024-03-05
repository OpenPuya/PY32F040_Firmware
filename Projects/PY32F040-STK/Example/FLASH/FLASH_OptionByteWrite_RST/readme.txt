================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过软件方式将RESET引脚改为普通GPIO。

Function descriptions:
This example demonstrates changing the RESET pin to regular GPIO through software.
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
2. 请将MCU断电并重新上电；
3. 按下按键，RST键变为GPIO功能，LED亮；
4. 注释掉"RSTPIN_MODE_GPIO",打开"RSTPIN_MODE_RST",编译重新下载到MCU;
5. 将MCU断电并重新上电；
6. 按下按键，RST键变为RESET功能，LED亮；

Example execution steps:
1. Compile and download the program to MCU and run it;
2. Please turn off the MCU and power it back on;
3. Press the button,RST key changes to GPIO function and LED lights up;
4. Annotate "RSTPIN_MODE_GPIO" and open "RSTPIN_MODE_RST",
Compile and download again to MCU;
5. Turn off the MCU and power it back on;
6. Press the button,The RST key changes to the RESET function, and the LED lights
up;
================================================================================
注意事项：
该样例将RST的按键改成GPIO功能，在使用其他样例前请将RST按键改回RST功能。

Notes:
This example changes the RST button to GPIO function, please change the RST button
back to RST function before using other examples.
================================================================================