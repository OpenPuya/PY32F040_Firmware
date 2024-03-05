================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了TIM1的输出比较模式。将捕获/比较通道1(CH1)的输出映射到PA8，开启捕获
/比较通道1(CH1)并设置为比较输出翻转模式

Function descriptions:
This sample demonstrates the output compare mode of TIM1. The output of 
capture/compare channel 1 (CH1) is mapped to pin PA8. Capture/compare 
channel 1 (CH1) is enabled and set to compare output toggle mode. 
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
1. 编译下载程序到MCU，并运行
2. 使用示波器观察PA8，频率为0.5Hz

Example execution steps:
1.Compile and download the program to the MCU, and then run it.
2.Use an oscilloscope to observe pin PA8. The frequency should be 5Hz.
================================================================================
注意事项：

Notes:

================================================================================