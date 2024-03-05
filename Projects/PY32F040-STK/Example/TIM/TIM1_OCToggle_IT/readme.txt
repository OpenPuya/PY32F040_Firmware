================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了TIM1比较模式下的中断功能，在中断中翻转GPIO。

Function descriptions:
This sample demonstrates the interrupt function in TIM1 comparison mode, toggle 
the GPIO in an interrupt.
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
1. 编译下载并运行程序；
2. 逻辑分析仪抓取PA4/PA5波形,PA4在比较中断中翻转，PA5在更新中断中翻转。

Example execution steps:
1. compile and download the program to MCU and run it;
2. Using logic analyzer captures the PA4/PA5 waveform,PA4 toggle in the 
comparison interrupt and PA5 toggle in the update interrupt.
================================================================================
注意事项：

Notes:

================================================================================