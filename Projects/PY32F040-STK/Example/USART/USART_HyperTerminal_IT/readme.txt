================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了USART的中断方式发送和接收数据，USART配置为115200，数据位8，停止位1，
校验位None,下载并运行程序后，上位机通过USART会接收到0x1-0xC,然后通过上位机下
发12个数据，例如0x1~0xC,则，MCU会把接收到的数据再次发送到上位机。

Function descriptions:
This example demonstrates how to use USART to send an amount of data in interrupt
mode USART configuration is 115200 baud rate, data bit 8, stop bit 1, check bit 
None. After download and run the program,the upper computer will receive 0x1-0xC
Then send 12 data(such as 0x1~0xC)by the host computer,and the MCU will send the 
received data again.
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
1. 编译并下载程序到MCU；
2. 通过USB转TTL模块连接PC与STK板,STK板与USB转TTL模块的连线方式如下；
STK板        USB转TTL模块
PA02(TX) --> RX
PA03(RX) --> TX
GND      --> GND
3. PC端打开串口调试助手，正确连接上通讯COM口，波特率设置为115200；
4. MCU会发送0x1~0xc到PC端
5  上位机接收后. 上位机发送12个数据，MCU会反馈同样的12个数据给上位机

Example execution steps:
1. compile and download the program to MCU and run it;
2. Connect the PC to the STK board through the USB-to-TTL module. The connection 
method between the STK board and the USB-to-TTL module is as follows.
STK board    USB-to-TTL module
PA02(TX) --> RX
PA03(RX) --> TX
GND      --> GND
3. Start the serial assistant on the PC, correctly connect the COM port, and set
the baud rate to 115200.
4. The MCU will send 0x1~0xc to the PC
5. After received,the host computer sends 12 data and the MCU will return the 
same 12 data to the host computer
================================================================================
注意事项：

Notes:

================================================================================