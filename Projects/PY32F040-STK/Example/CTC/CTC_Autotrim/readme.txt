================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了CTC使用LSE做参考时钟自动校准PLL48M时钟的功能。

Function descriptions:
This example demonstrates the function of CTC using LSE as a reference clock to
automatically calibrate the PLL48M clock.
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
1. 编译并下载程序；
2. 运行程序后，观察LED灯是否亮起，如果亮起说明时钟校准成功,使用示波器观察PA8，
输出稳定48M频率脉冲；否则时钟校准未成功。

Example execution steps:
1. Compile and download the program;
2. After running the program, observe whether the LED light is on. If it is on,
it indicates successful clock calibration. Use an oscilloscope to observe PA8
and output a stable 48M frequency pulse; Otherwise, clock calibration is not
successful.
================================================================================
注意事项：

Notes:

================================================================================