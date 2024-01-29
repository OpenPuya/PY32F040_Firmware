================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1中使用DMA连续两次burst传输数据的功能，burst每传输一次更新三
个寄存器，PSC,ARR,RCR，在更新事件中断中，LED会闪烁，通过逻辑分析仪监测，可看到
PA1的翻转间隔会从第一次的2s，第二次2s，第三次1s,第四次及后续变为0.5s，此时两次
burst传输完成，并且PCS,ARR,RCR均更新完毕。

Function descriptions:
This sample demonstrates the function to transfer data in TIM1 using DMA in two 
consecutive bursts.burst updates three registers(PSC,ARR,RCR) per transfer.In 
the interruption of update event, LED will be blinked. Through the monitoring of
logic analyzer, it can be seen that the flipping interval of PA1 will change 
from 2s for the first time, 2s for the second time, 1s for the third time, and 
0.5s for the fourth and subsequent times. At this time, the two burst
transmission is completed, and PCS、ARR and RCR are all updated.
================================================================================
测试环境：
测试用板：PY32F403_STK
MDK版本： 5.28
IAR版本： 9.20

Test environment:
Test board: PY32F403_STK
MDK Version: 5.28
IAR Version: 9.20
================================================================================
使用步骤：
1. 下载并运行程序
2. 观察LED闪烁频率越来越快

Example execution steps:
1. compile and download the program to MCU and run it;
2. Observe the LED flashing frequency getting faster and faster
================================================================================
注意事项：

Notes:

================================================================================