================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了FreeRTOS信号量的相关API函数，实现二值信号量、计数型信号量和互斥信号
量三种信号量实验，通过串口log，观察三种信号量的实现过程。

Function descriptions:
This sample demonstrates the relevant API functions of FreeRTOS signals to 
implement three kinds of signaling experiments: binary signals, counting-type 
signals and mutually exclusive signals, and observes the implementation process 
of the three kinds of signals through the serial port log.

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
3. 打开宏定义SEMAPHORE_BINRAY，关闭SEMAPHORE_COUNT和SEMAPHORE_MUTEX。
   Task1: 任务每隔2s释放一次二值信号量。
   Task2: 获取二值信号量，打印获取成功信息。获取失败则进入等待状态。
   Task3: 删除任务3。
4. 打开宏定义SEMAPHORE_COUNT，关闭SEMAPHORE_BINRAY和SEMAPHORE_MUTEX。
   Task1: 任务每隔2s释放一次计数型信号量。信号量总数加1。总数上限是50。
   Task2: 获取计数型信号量，打印获取成功信息。信号量总数减1。
   Task3: 删除任务3。
5. 打开宏定义SEMAPHORE_MUTEX，关闭SEMAPHORE_BINRAY和SEMAPHORE_COUNT。
   Task1: 任务每隔4s(3s delay延时 + 1s阻塞延时)获取和释放互斥信号量。
   Task2: 打印任务2运行log。
   Task3: 任务每隔1.5s(0.5s delay延时 + 1s阻塞延时)获取和释放互斥信号量。
   步骤5：是为了验证实现互斥信号量优先级继承功能。

Example execution steps:
1. Compile and download the program to the MCU and run it;
2. You can use the log information printed by the serial debugging assistant to 
observe the task execution process.
3. Open the define SEMAPHORE_BINRAY, close SEMAPHORE_COUNT and SEMAPHORE_MUTEX.
   Task1: The task releases a binary semaphore every 2s.
   Task2: Get a binary semaphore and print a success message. If the acquisition
          fails, the system enters the waiting state.
   Task3: Delete Task 3.
4. Open the define SEMAPHORE_COUNT, close SEMAPHORE_BINRAY and SEMAPHORE_MUTEX.
   Task1: The task releases a counting semaphore every 2s. Add 1 to the total 
          number of semaphores. The total number is capped at 50.
   Task2: Obtain the counting semaphore and print the obtaining success message. 
          Subtract 1 from the total semaphore.
   Task3: Delete Task 3.
5. Open the define SEMAPHORE_MUTEX, close SEMAPHORE_BINRAY and SEMAPHORE_COUNT.
   Task1: The task obtains and releases mutually exclusive semaphore every 4s
          (3s delay + 1s blocking delay).
   Task2: Print Task 2 Run log.
   Task3: The task acquires and releases mutually exclusive semaphore every 
          1.5s(0.5s delay + 1s blocking delay).
   Step 5: Is to verify the implementation of the MUtex priority inheritance 
           function.

================================================================================
注意事项：
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
