================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS列表的相关API函数应用，实现列表项的插入和删除。通过串口log，
观察列表项的插入和删除实现过程。

Function descriptions:
This sample demonstrates the application of FreeRTOS list related API functions 
to realize the insertion and deletion of list items. Observe the process of 
implementing the insertion and deletion of list items through the serial port log.

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
   Task1: 每隔500ms打印一次"Task1 is running"信息。
   Task2: 创建列表、列表项，插入、删除，打印列表项地址信息。每隔1s打印一次"Task2
          is running"信息。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
observe the task execution process every 500ms.
   Task1: Print the "Task1 is running" message.
   Task2: Create lists, list items, insert, delete, and print list item address 
          information.Print the "Task2 is running" message every 1s.

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
