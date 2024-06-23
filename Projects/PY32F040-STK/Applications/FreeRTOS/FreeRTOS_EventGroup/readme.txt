================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS事件标志组的相关API函数应用，实现事件标志组的设置和读取。通
过串口log，观察事件标志组设置和读取实现过程。

Function descriptions:
This sample demonstrates the application of FreeRTOS event flag group related 
API functions to realize the setting and reading of event flag group. Observe 
the process of setting and reading the event flag group through the serial port 
log.

================================================================================
测试环境：
测试用板：PY32F040_STK
MDK版本：5.28
IAR版本：9.20
GCC版本：GNU Arm Embedded Toolchain 10.3-2021.10

Test environment:
Test board: PY32F040_STK
MDK Version: 5.28
IAR Version: 9.20
GCC Version: GNU Arm Embedded Toolchain 10.3-2021.10

================================================================================
使用步骤：
1. 编译下载程序到MCU，并运行；
2. 通过串口调试助手打印的log信息，观察任务执行过程。
   Task1: 任务运行1s后，设置事件标志组值的bit0位为1。任务运行2s后，设置事件标志
   组值的bit1位为1。计时清零，重复这两个步骤。
   Task2: 读取并打印事件标志组值。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
observe the task execution process.
   Task1: After the task runs for 1s, the bit0 bit of the event flag 
          group value is set to 1. After the task runs for 2s, set the bit1 bit
          of the event flag group value to 1. Time clear and repeat the two steps.
   Task2: Reads and prints event flag group values.

================================================================================
注意事项：
STK板与USB转TTL模块的连线方式如下：
STK板        USB转TTL模块
PA02(TX) --> RX
PA03(RX) --> TX
GND      --> GND
波特率：115200bps

Notes:
The STK board is wired to the USB to TTL module as follows:
STK board    USB to TTL module
PA02(TX) --> RX
PA03(RX) --> TX
GND      --> GND
Baud rate: 115200bps
================================================================================
