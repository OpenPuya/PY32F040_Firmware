================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了ADC模块的VCC采样功能，通过采样VREFINT的值，计算得出VCC的值，并通过
串口打印出来。

Function descriptions:
This example demonstrates the VCC sampling function of the ADC module. By
sampling the value of VREFINT, the VCC value is calculated and printed through
the serial port.
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
1. 编译并下载程序；
2. 串口每隔1s打印一次VCC的电压值。

Example execution steps:
1. Compile and download the program;
2. The serial port prints the voltage value of VCC every 1 second.
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

STK board USB to TTL module
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND -->GND
================================================================================