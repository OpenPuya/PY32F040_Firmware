================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了将系统时钟设置为LSI，并通过MCO引脚输出系统时钟。

Function descriptions:
This example demonstrates setting the system clock to LSI and outputting the
system clock through the MCO pin.
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
2. 可由示波器观察到PA08输出LSI波形(32.768KHz)

Example execution steps:
1. Compile and download the program to the MCU.
2. Use an oscilloscope to observe the LSI waveform (32.768KHz) on pin PA08.
================================================================================
注意事项：

Notes:
================================================================================
