================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了硬件除法器计算无符号除法。

Function descriptions:
This example demonstrates how a hardware divider calculates unsigned division.
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
2. 串口打印计算结果，商为199842，余数为3809。

Example execution steps:
1.Compile and download the program to MCU and run it;
2.The serial port prints the calculation result, with a quotient of 199842 
and a remainder of 3809.
================================================================================
注意事项：
1.  STK板        USB转TTL模块
     PA02(TX) --> RX
     PA03(RX) --> TX
     GND      --> GND
     波特率:115200

Notes:
1.  STK board    USB to TTL module
     PA02(TX) --> RX
     PA03(RX) --> TX
     GND      --> GND
     Baud rate: 115200
================================================================================