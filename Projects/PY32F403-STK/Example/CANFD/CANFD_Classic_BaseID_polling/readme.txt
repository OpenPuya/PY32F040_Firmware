================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了采用CAN2.0协议标准帧轮询方式与PCAN-View的通信功能，MCU首先自动向
PCAN-View发送8byte数据0x1~0x8，PCAN-View接收到数据后，然后手动通过PCAN-View向MCU
发送ID为0x12F的8byte数据，MCU会自动将接收到数据通过串口打出。

Function descriptions:
This example demonstrates the communication function with PCAN View using the
CAN2.0 protocol standard frame polling method. The MCU first automatically sends
8-byte data 0x1~0x8 to PCAN View. After PCAN View receives the data, it manually
sends 8-byte data with ID 0x12F to the MCU through PCAN View. The MCU will
automatically print the received data through the serial port.
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
3. 通过USB转CAN模块连接PC、CAN转接板、STK板,连线方式如下；
STK板               CAN转接板
PA11(CAN_RX)  -->    R21  
PA12(CAN_TX)  -->    R20
GND           -->    GND

CAN转接板           USB转CAN模块
CAN-H         -->    CAN-H
CAN-L         -->    CAN-L
4. 打开PCAN-View，配置PCAN-View连接方式为：CAN2.0，Clock Frequency：20MHz，
Nominal Bit Rate：250kBit/s，Data Bit Rate：不设置，Standard
5. 复位STK板，观察PCAN-View是否接收到8byte数据数据0x1~0x8，PCAN-View接收到表明
STK板发送数据成功，否则表明STK板发送数据失败；
6. PCAN-View接收到数据后，手动通过PCAN-View发送ID为0x12F的8byte数据，观察串口是
否打印PCAN-View发送的数据，如果串口打印，表明STK板接收数据成功，否则STK板接收数
据失败。

Example execution steps:
1. Compile and download the program.
2. Connect the PC and STK board with a USB-to-TTL module. The connections 
between the STK board and the USB-to-TTL module are as follows:
STK board USB-to-TTL module
PA02(TX) --> RX
PA03(RX) --> TX
GND --> GND
3. Connect the PC, CAN adapter board, and STK board through the USB to CAN module,
and the connection method is as follows:
STK board           CAN adapter board
PA11(CAN_RX)    -->    R21  
PA12(CAN_TX)    -->    R20
GND             -->    GND

CAN adapter board     USB to CAN module
CAN-H           -->    CAN-H
CAN-L           -->    CAN-L
4. Open PCAN-View and configure the connection method for PCAN-View as CAN2.0,
Clock Frequency: 20MHz, Nominal Bit Rate: 250kBit/s, Data Bit Rate: not set,
Standard
5. Reset the STK board and observe if PCAN-View has received 8-byte data data
0x1~0x8. PCAN-View receiving indicates that the STK board has successfully sent
data, otherwise it indicates that the STK board has failed to send data;
6. After receiving the data, PCAN-View manually sends an 8-byte data with ID
0x12F through PCAN-View. Observe whether the serial port prints the data sent
by PCAN-View. If the serial port prints, it indicates that the STK board has
successfully received the data, otherwise the STK board has failed to receive
the data.
================================================================================
注意事项：

Notes:

================================================================================