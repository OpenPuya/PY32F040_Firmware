================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS队列的相关API函数应用，实现队列的入队和出队。通过串口log，观
察任务出队和入队实现过程。

Function descriptions:
This sample demonstrates the application of FreeRTOS queue related API functions
to realize queue in and queue out. Through the serial port log, observe the 
process of task out and in queue implementation.

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
   Task1: 任务执行1s后向队列KeyQueue发送一个值，任务执行2s后向队列BigDataQueue发
          送一个数组首地址。计时清零，并重复这两个步骤。
   Task2: 从队列KeyQueue接收一个值，接收成功，打印该值信息。
   Task3: 从队列BigDataQueue接收一组值，接收成功，打印该组值信息。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
observe the task execution process.
   Task1: A value is sent to the KeyQueue after the task is executed for 1s, and
          the first address of an array is sent to the BigDataQueue after 2s. 
          Time the reset and repeat the two steps.
   Task2: A value is received from the KeyQueue. If the value is received 
          successfully, the value information is printed.
   Task3: A set of values is received from the BigDataQueue. If the data is 
          received successfully, the set of values is printed.

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
