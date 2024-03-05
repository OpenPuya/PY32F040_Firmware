================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了在TIM1中使用DMA连续两次burst传输数据的功能，burst每传输一次更新三
个寄存器，PSC,ARR,RCR，在更新事件中断中，PA0会进行翻转，通过逻辑分析仪监测，
可看到PA0的翻转间隔会从第一次的400ms，第二次400ms，第三次20ms,第四次及后续变为
200us，此时两次burst传输完成，并且PCS,ARR,RCR均更新完毕。

Function descriptions:
This sample demonstrates the function to transfer data in TIM1 using DMA in two 
consecutive bursts.burst updates three registers(PSC,ARR,RCR) per transfer.In 
the interruption of update event, PA0 will be flipped. Through the monitoring of
logic analyzer, it can be seen that the flipping interval of PA0 will change 
from 400ms for the first time, 400ms for the second time, 20ms for the third 
time, and 200us for the fourth and subsequent times. At this time, the two burst
transmission is completed, and PCS,ARR and RCR are all updated.
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
3. 翻转间隔从400ms,接着变为2ms,最后变为200us
4. 变为200us后，翻转间隔均为200us

Example execution steps:
1. compile and download the program to MCU and run it;
2. Capture the turnover interval on PA0 through the logic analyzer oscilloscope
3. The turnover interval changed from 400ms, then to 2ms, and finally to 200us
4. After the value is changed to 200us, the turnover interval is always 200us
================================================================================
注意事项：

Notes:

================================================================================