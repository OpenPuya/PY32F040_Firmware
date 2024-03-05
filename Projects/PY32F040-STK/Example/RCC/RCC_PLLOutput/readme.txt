================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例配置系统时钟为PLL，并通过MCO（PA08）引脚输出。PLL的输入时钟源选择HSI。

Function descriptions:
This sample configures the system clock as PLL and outputs it through the MCO 
(PA08) pin. The PLL input clock source is set to HSI.
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
2. 按下用户按键；
3. 用示波器监测PA08引脚上的频率，观察是否和配置的PLL频率（32MHz）一致。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Press the user button.
3. Use an oscilloscope to monitor the frequency on the PA08 pin and check if it 
   matches the configured PLL frequency (32MHz).
================================================================================
注意事项：
1. 注意PLL的输入时钟源频率在2倍频时必须大于等于12MHz，3倍频时必须大于等于16MHz。

Notes:
1. Note that the PLL input clock source frequency must be greater than or equal 
   to 12MHz for 2x PLL multiplication, and greater than or equal to 16MHz for 
   3x PLL multiplication.
================================================================================
