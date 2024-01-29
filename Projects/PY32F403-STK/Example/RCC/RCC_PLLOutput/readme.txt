================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例配置系统时钟为PLL，并通过MCO（PA08）引脚输出，PLL的输入时钟源选择HSE。

Function descriptions:
This sample configures the system clock to use the PLL (Phase-Locked Loop) 
clock source with HSE (High-Speed External) as the input clock source and 
outputs it through the MCO (PA08) pin. The PLL frequency is configured to 
be 144MHz.

================================================================================
测试环境：
测试用板：PY32F403_STK
MDK版本：5.28
IAR版本：9.20

Test environment:
Test board: PY32F403_STK
MDK Version: 5.28
IAR Version: 9.20

================================================================================
使用步骤：
1. 编译下载程序到MCU，并运行；
2. 按下用户按键；
3. 使用示波器监测PA08引脚上的频率，观察是否与配置的PLL频率（144MHz）一致；

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. Press the user button;
3. Use an oscilloscope to monitor the frequency on the PA08 pin and check if 
it matches the configured PLL frequency (144MHz).

================================================================================
注意事项：

Notes:

================================================================================
