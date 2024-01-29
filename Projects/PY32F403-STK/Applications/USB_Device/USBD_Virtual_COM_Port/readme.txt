================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了USB虚拟串口功能（无UART功能）。
USB虚拟串口主要通过USB CDC ACM类实现，将其模拟成一个VCP设备，当插在电脑上时，
可以显示成一个串口设备。跟市面上的USB2TTL模块的区别在于，此样例只使用USB，没有与
串口（UART外设）进行连动。

Function descriptions:
This sample demonstrates the USB virtual serial port (VCP) functionality (without 
UART). The USB VCP is mainly implemented using the USB CDC ACM class, simulating 
a VCP device. When connected to a computer, it appears as a serial port device. 
The difference from the USB2TTL module on the market is that this sample only 
uses USB and does not interact with a UART peripheral.

================================================================================
测试环境：
测试用板：STK
MDK版本：5.28

Test environment:
Test board: STK
MDK Version: 5.28

================================================================================
使用步骤：
1. 下载并运行程序；
2. 通过USB连接线，一端连接STK板，另一端连接PC电脑；
3. 打开PC电脑上的设备管理器，可以发现多了一个COM口；
4. 打开串口调试助手，开启对应COM口，并使能DTR，串口调试助手每隔1秒会收到128个
   字节数据；
5. PC端串口发送一串数据（不超过128个字节）到设备，MCU通过debug模式查看
   read_buffer数组，发现数据和PC端发送的数据一致。

Example execution steps:
1. Download and run the program;
2. Connect the STK board to the PC computer using the USB cable;
3. Open the Device Manager on the PC, and you will find an additional COM port;
4. Open the serial port debugging assistant, enable the corresponding COM port, 
   and enable DTR. The serial port debugging assistant will receive 128 bytes 
   of data every 1 second;
5. Send a string of data (up to 128 bytes) from the PC to the device. Through 
   the debug mode, the MCU can view the read_buffer array and find that the 
   data matches the data sent by the PC.

================================================================================
注意事项：
1. 用户测试程序过程中，如果对USB接口进行热插拔，当重新插上USB线到PC端后，有可能
   串口助手收不到数据。这是因为程序每次发送数据之前会判断上一次是否发送成功（检
   查标志位ep_tx_busy_flag）。如果热插拔过程中导致USB通讯失败一次，则设备会停止
   向主机发送数据。您可以通过在程序中取消对ep_tx_busy_flag标志位的判断，即可实
   现热插拔不影响串口助手数据接收。

Notes:
1. During user testing, if the USB interface is hot-plugged, and after 
   re-plugging the USB cable to the PC, the serial port assistant may not 
   receive data. This is because before sending data, the program checks 
   if the previous transmission was successful (check the ep_tx_busy_flag 
   flag). If there is a USB communication failure during the hot-plug process, 
   the device will stop sending data to the host. You can implement hot-plug 
   without affecting the serial port assistant data reception by removing 
   the check for the ep_tx_busy_flag flag in the program.

================================================================================
