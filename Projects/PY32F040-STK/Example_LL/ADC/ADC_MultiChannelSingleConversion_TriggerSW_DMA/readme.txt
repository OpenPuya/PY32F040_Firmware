================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了ADC的DMA多通道传输功能，在DMA完成中断中打印多通道的电压值。

Function descriptions:
This example demonstrates the DMA multi-channel transmission function of ADC,
which prints the voltage values of multiple channels during the DMA completion
interrupt.
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
2. 串口每隔1s打印一次通道4(PA4)/通道5(PA5)/通道6(PA6)/通道7(PA7)的电压值。

Example execution steps:
1. Compile and download the program and run it;
2. The serial port prints the voltage values of channel 4 (PA4)/channel 5 (PA5)/
channel 6 (PA6)/channel 7 (PA7) every 1 second.
================================================================================
注意事项：
通过USB转TTL模块连接PC与STK板,STK板与USB转TTL模块的连线方式如下；
STK板        USB转TTL模块
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND

Notes:
Connect the PC to the STK board through the USB to TTL module, the connection
between the STK board and the USB to TTL module is as follows:
STK board      USB to TTL module
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND        --> GND
================================================================================