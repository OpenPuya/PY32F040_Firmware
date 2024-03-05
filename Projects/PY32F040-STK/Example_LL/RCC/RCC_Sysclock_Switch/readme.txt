================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了时钟切换，由LSI（32.768KHz）切换至HSE（24MHz）。

Function descriptions:
This sample demonstrates clock switching from LSI (32.768KHz) to HSE (24MHz).
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
2. 用示波器监测PA08引脚上的频率，MCU启动后一开始输出LSI时钟，频率是32.768KHz，按
   下用户键后，输出HSE时钟（24MHz）。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Monitor the frequency on pin PA08 using an oscilloscope. After the MCU 
starts up, it initially outputs the LSI clock with a frequency of 32.768KHz. 
Press the user button to switch to the HSE clock (24MHz).
================================================================================
注意事项：

Notes:
================================================================================
