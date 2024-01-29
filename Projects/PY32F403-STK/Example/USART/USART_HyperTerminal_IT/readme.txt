================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了USART的中断方式发送和接收数据，USART配置为115200，数据位8，停止位1，
校验位None，下载并运行程序后，上位机通过USART会接收到0x1-0xC，然后通过上位机下
发12个数据，例如0x1~0xC，则MCU会把接收到的数据再次发送到上位机。

Function descriptions:
This sample demonstrates the USART data transmission and reception using the 
interrupt method. The USART is configured with a baud rate of 115200, 8 data 
bits, 1 stop bit, and no parity. After compiling and downloading the program 
to the MCU, the host computer will receive data from 0x1 to 0xC through USART. 
Then, when the host computer sends 12 data bytes (0x1 to 0xC), the MCU will 
send back the received data to the host computer.

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
1. 编译并下载程序到MCU；
2. 通过USB转TTL模块连接PC与STK板，STK板与USB转TTL模块的连线方式如下：
   STK板        USB转TTL模块
   PA02(TX) --> RX
   PA03(RX) --> TX
   GND      --> GND
3. PC端打开串口调试助手，正确连接上通讯COM口，波特率设置为115200；
4. MCU会发送0x1~0xC到PC端；
5. 上位机接收后，上位机发送12个数据，MCU会反馈同样的12个数据给上位机。

Example execution steps:
1. Compile and download the program to the MCU;
2. Connect the PC and STK board with a USB to TTL module. The connection between 
   the STK board and the USB to TTL module is as follows:
   STK board        USB to TTL module
   PA02(TX) --> RX
   PA03(RX) --> TX
   GND      --> GND
3. Open a serial debugging assistant on the PC and connect to the appropriate COM 
   port with the baud rate set to 115200;
4. The MCU will send data from 0x1 to 0xC to the PC;
5. After receiving data on the host computer, the host computer sends 12 data 
   bytes (0x1 to 0xC), and the MCU will send back the same 12 data bytes to the 
   host computer.

================================================================================
注意事项：

Notes:

================================================================================
