================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1中使用DMA传输数据的功能，通过DMA从SRAM中搬运数据到ARR寄存器，
实现TIM1周期变化，在TIM1第一次溢出后，PA0会翻转，此时翻转间隔为400ms，DMA开始
搬运数据到TIM1_ARR，第一次PA0翻转间隔为400ms，第二次翻转间隔为100ms，第三次翻
转间隔为200ms，第四次翻转间隔为300ms，此时DMA搬运结束，后续翻转间隔均为300ms

Function descriptions:
This sample demonstrates the function of using DMA to transfer data in TIM1, 
carrying data from SRAM to ARR register by DMA to achieve TIM1 cycle change. 
After the first overflow of TIM1, PA0 will toggle, at this time the toggle 
interval is 400ms. DMA starts to carry data to TIM1_ARR, the first PA0 toggle 
interval is 400ms, the second toggle interval is 100ms, the third toggle 
interval is 200ms, the fourth toggle interval is 300ms, at this time the DMA 
carrying ends, the subsequent toggle interval are 300ms
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
1. 下载并运行程序
2. 通过逻辑分析仪示波器捕捉PA0上翻转间隔
3. 翻转间隔从400ms，变为100ms，200ms，300ms
4. 变为300ms后，翻转间隔均为300ms

Example execution steps:
1. compile and download the program to MCU and run it;
2. Capture the toggle interval on PA0 through the logic analyzer oscilloscope
3. The toggle interval is changed from 400ms to 100ms, 200ms, 300ms
4. After the value is changed to 300ms, the toggle interval is 300ms
================================================================================
注意事项：

Notes:

================================================================================