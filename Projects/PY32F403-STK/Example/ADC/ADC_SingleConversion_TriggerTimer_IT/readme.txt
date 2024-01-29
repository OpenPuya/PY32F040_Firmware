================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了ADC通过TIM触发转换的功能，每隔1s，TIM触发ADC转换一次，并通过串口打印
出来。

Function descriptions:
This example demonstrates the function of ADC triggering conversion through TIM.
Every 1 second, TIM triggers ADC conversion and prints it out through the serial
port.
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
1. 编译并下载程序；
2. 串口每隔1s打印一次通道4的DR值；

Example execution steps:
1. Compile and download the program;
2. The serial port prints the DR value of channel 4 every 1 second;
================================================================================
注意事项：
通过USB转TTL模块连接PC与STK板,STK板与USB转TTL模块的连线方式如下；
STK板        USB转TTL模块
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND

Notes:
Connect the PC to the STK board through the USB to TTL module, and the connection
method between the STK board and the USB to TTL module is as follows:

STK board     USB to TTL module
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND
================================================================================