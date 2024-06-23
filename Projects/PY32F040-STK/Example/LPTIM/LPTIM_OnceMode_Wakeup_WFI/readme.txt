================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了LPTIM单次模式中断唤醒STOP模式。

Function descriptions:
This example demonstrates the LPTIM once mode interrupt wake-up STOP mode.
================================================================================
测试环境：
测试用板：PY32F040_STK
MDK版本：5.28
IAR版本： 9.20
GCC 版本：GNU Arm Embedded Toolchain 10.3-2021.10

Test environment:
Test board: PY32F040_STK
MDK Version: 5.28
IAR Version: 9.20
GCC Version: GNU Arm Embedded Toolchain 10.3-2021.10
================================================================================
使用步骤：
1. 编译并下载程序到MCU，并运行。
2. 按下按键进入STOP模式。
3. 200毫秒后唤醒STOP模式并循环进入STOP模式再唤醒。
4. LPTIM中断中LED以2.5hz闪烁，while中PB1以2.5hz翻转；

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Press the button to enter STOP mode.
3. After 200 milliseconds, wake up from STOP mode and repeat the process of 
   entering STOP mode and waking up.
4. In the LPTIM interrupt, the LED flashes at 2.5Hz, while in the while function,
PB1 flips at 2.5Hz;
================================================================================
注意事项：

Notes:
================================================================================
