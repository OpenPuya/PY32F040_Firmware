================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS任务通知的相关API函数应用，实现模拟信号量、模拟事件标志组、
模拟消息邮箱功能。通过串口log，观察任务通知模拟的实现过程。

Function descriptions:
This sample demonstrates the application of FreeRTOS task notification related 
API functions to implement the simulation of semaphores, simulation of event flag
 groups, and simulation of message mailbox functions. Observe the implementation 
 process of task notification simulation through serial port log.

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
3. 打开宏定义NOTIFY_SEMAPHORE，关闭NOTIFY_NOTIFY_EVENTGROUPS和NOTIFY_MESSAGE。
   Task1: 任务通知每隔1s向任务2释放一个信号量。信号量总数加1。
   Task2: 任务2每隔2s获取一次信号量，获取成功，打印信号量的值。信号量总数减1。
4. 打开宏定义NOTIFY_NOTIFY_EVENTGROUPS，关闭NOTIFY_SEMAPHORE和NOTIFY_MESSAGE。
   Task1: 任务运行1s后向任务2的事件标志组值bit0位写1，任务运行2s后向任务2的事件
          标志组值bit1位写1。计时清零，并重复这两个步骤。
   Task2: 任务2获取事件标志组值，获取成功后，打印该值。获取失败则进入等待状态。
5. 打开宏定义NOTIFY_MESSAGE，关闭NOTIFY_SEMAPHORE和NOTIFY_NOTIFY_EVENTGROUPS。
   Task1: 任务运行1s后给任务2的通知值赋值10，任务运行2s后向任务2的通知值赋值20。
          计时清零，并重复这两个步骤。
   Task2: 任务2获取任务通知存放的值，获取成功后，打印该值。获取失败则进入等待状
          态。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
   observe the task execution process.
3. Open the define NOTIFY SEMAPHORE, close the NOTIFY NOTIFY EVENTGROUPS and 
   NOTIFY MESSAGE.
   Task1: The task notification releases a semaphore to task 2 every 1s. Add 1 
          to the total number of semaphores.
   Task2: Task 2 obtains the semaphore every 2s, obtains the semaphore 
          successfully, and prints the semaphore value. Subtract 1 from the 
          total semaphore.
4. Open the define NOTIFY_NOTIFY_EVENTGROUPS, close the NOTIFY_SEMAPHORE and 
   NOTIFY MESSAGE.
   Task1: The event flag group value bit0 is written to 1 after the task is 
          run for 1s, and the event flag group value bit1 is written to 1 after
          the task is run for 2s.Time the reset and repeat the two steps.
   Task2: Task 2 Obtain the event flag group value and print the value. If 
          the acquisition fails, the system enters the waiting state.
5. Open the define NOTIFY MESSAGE, close the NOTIFY_SEMAPHORE and 
   NOTIFY_NOTIFY_EVENTGROUPS.
   Task1: A value of 10 is assigned to the notification value of task 2 after 
          the task runs for 1s, and a value of 20 is assigned to the 
          notification value of task 2 after the task runs for 2sTime the 
          reset and repeat the two steps.
   Task2: Task 2 Obtain the value saved in the task notification and print 
          the value. If the acquisition fails, the system enters the waiting 
          state.

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
