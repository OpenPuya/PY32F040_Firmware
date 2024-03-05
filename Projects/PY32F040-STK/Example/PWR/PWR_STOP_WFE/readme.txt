================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了stop模式下，通过GPIO事件唤醒功能。

Function descriptions:
This sample demonstrates the stop mode with GPIO event wake-up feature.
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
2. LED灯亮，按下按键，LED灯灭，串口打印"STOP MODE!"，进入stop模式；
3. 在PA06上产生一个下降沿，程序退出stop模式，并且串口打印"WAKEUP OK!"；
4. LED以200ms的间隔进行翻转。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. The LED is on, press the button, the LED turns off, and "STOP MODE!" is printed 
   on the serial port, entering the stop mode.
3. Generate a falling edge on PA06 to wake up the program from stop mode, and 
   "WAKEUP OK!" will be printed on the serial port.
4. The LED will toggle with an interval of 200ms.
================================================================================
注意事项：
MCU与串口调试助手连线:
RX ---> PA02
TX ---> PA03
波特率：115200bps

1. 演示此样例功能时需要断开swd连接线并重新上电，因为默认情况下，仿真器会把
   DBGMCU_CR.DBG_STOP置位。
2. 注意关闭systick中断，防止该中断唤醒MCU。

Notes:
MCU to Serial Debug Assistant connection:
RX ---> PA02
TX ---> PA03
Baud rate: 115200bps

1. When demonstrating this sample, it is necessary to disconnect the SWD 
   connection and power on again because the debugger will set 
   DBGMCU_CR.DBG_STOP by default.
2. Pay attention to disable the systick interrupt to prevent it from 
   waking up the MCU.
================================================================================
