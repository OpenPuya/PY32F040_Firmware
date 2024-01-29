================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了采用CANFD协议、扩展帧、外部回环的轮询方式与PCAN-View的通信功能，MCU
首先自动向发送64byte数据0x0~0x3f，MCU接收到数据后，自动将接收到数据通过串口打出。

Function descriptions:
This example demonstrates the communication function with PCAN-View using the
CANFD protocol, extended frames, and external loopback polling method. The MCU
first automatically sends 64byte data 0x0~0x3f to the. After receiving the data,
the MCU automatically prints the received data through the serial port.
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
2. 通过USB转TTL模块连接PC与STK板,STK板与USB转TTL模块的连线方式如下；
STK板        USB转TTL模块
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND
3. 将CAN_RX（PA11）与CAN_TX（PA12）短接；
4. 复位STK板，观察串口是否打印64byte数据0x0~0x3f，如果串口打印，表明外部回环模式
测试成功。

Example execution steps:
1. Compile and download the program.
2. Connect the PC and STK board with a USB-to-TTL module. The connections 
between the STK board and the USB-to-TTL module are as follows:
STK board USB-to-TTL module
PA02(TX) --> RX
PA03(RX) --> TX
GND --> GND
3. Connect CAN_ RX (PA11) and CAN_ TX (PA12) short circuit;
4. Reset the STK board and observe whether the serial port prints 64byte data
0x0~0x3f. If the serial port prints, it indicates that the external loop back
mode test is successful.
================================================================================
注意事项：

Notes:

================================================================================