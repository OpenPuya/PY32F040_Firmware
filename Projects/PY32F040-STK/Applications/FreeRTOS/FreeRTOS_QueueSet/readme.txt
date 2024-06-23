================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS队列集的相关API函数应用，实现队列的入队和出队，信号量释放和
获取。通过打印log，观察出队/入队和释放/获取实现过程。

Function descriptions:
This sample demonstrates the application of FreeRTOS queue set related API 
functions to implement queue in and queue out, semaphore release and fetch. 
Observe the queue-out/queue-in and release/acquire implementations by printing 
a log.

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
   Task1: 任务执行1s后向队列发送一个值。任务执行2s后释放一个信号量，计时清零，并
          重复这两个步骤。
   Task2: 从队列集中获取任务句柄，任务句柄为队列句柄，打印队列中存放的数值。任务
          句柄是信号量句柄，打印“Take semaphore succuss!”信息。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
observe the task execution process.
   Task1: Sends a value to the queue after the task is executed for 1s. Release
          a semaphore after the task is executed for 2s, time clear, and repeat
          these two steps.
   Task2: Obtains the task handle from the queue set. The task handle is the queue
          handle and prints the value stored in the queue. The task handle is a 
          semaphore handle that prints "Take semaphore succuss!" Message.

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
