================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了USART的自动波特率检测功能。调试助手发送一个字符0x7F，MCU反馈字符
串：Auto BaudRate Test。

Function descriptions:
This sample demonstrates the automatic baud rate detection feature of USART. 
When the debugging assistant sends a character 0x7F, the MCU will respond with 
the string: "Auto BaudRate Test".

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
3. PC端打开串口调试助手，正确连接上通讯COM口，波特率设置为9600；
4. 调试助手发送一个字符0x7F，MCU反馈字符串：Auto BaudRate Test；

Example execution steps:
1. Compile and download the program to the MCU;
2. Connect the PC and STK board with a USB to TTL module. The connection between 
   the STK board and the USB to TTL module is as follows:
   STK board        USB to TTL module
   PA02(TX) --> RX
   PA03(RX) --> TX
   GND      --> GND
3. Open a serial debugging assistant on the PC and connect to the appropriate 
   COM port with the baud rate set to 9600;
4. The debugging assistant sends a character 0x7F, and the MCU responds with 
   the string "Auto BaudRate Test";

================================================================================
注意事项：
1. 不注释掉宏AUTO_BAUD_MODE0（main.c中），上位机发送1字节的波特率检测0x7F，如果
   MCU检测成功，返回字符：Auto BaudRate Test
2. 注释掉宏AUTO_BAUD_MODE0（main.c中），上位机发送1字节的波特率检测0x55，如果MCU
   检测成功，返回字符：Auto BaudRate Test

Notes:
1. If the AUTO_BAUD_MODE0 macro is not commented out (in main.c), when the host 
   computer sends a 1-byte baud rate detection signal 0x7F, the MCU will respond 
   with the string: "Auto BaudRate Test".
2. If the AUTO_BAUD_MODE0 macro is commented out (in main.c), when the host 
   computer sends a 1-byte baud rate detection signal 0x55, the MCU will respond 
   with the string: "Auto BaudRate Test".

================================================================================
