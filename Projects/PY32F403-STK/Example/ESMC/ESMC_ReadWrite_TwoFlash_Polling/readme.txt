================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例演示了ESMC在间接模式下的双Flash polling传输功能，对两个P25Q64的芯片进行擦
除，写入数据，读取数据，然后把读取的数据和写入的数据进行对比，数据正确则LED灯闪
烁，否则LED灯不闪烁。

Function descriptions:
This example demonstrates the dual Flash polling transmission function of ESMC
in indirect mode, erasing, writing data, reading data on two P25Q64 chips, and
then comparing the read data with the written data. If the data is correct, the
LED light will flash, otherwise the LED light will not flash.
================================================================================
测试环境：
测试用板：PY32F403_STK
MDK版本： 5.28
IAR版本： 9.20

Test environment:
Test board: PY32F403_STK
MDK Version: 5.28
IAR Version: 9.20
================================================================================
使用步骤:
1.编译下载程序，并运行，
2.可观察到LED灯闪烁

Example execution steps:
1. Compile and download the program, and run it,
2. LED flashing can be observed
================================================================================
注意事项:
接线方式:
P25Q64_1         MCU
CLK         ->   PB10
SS0（CS）   ->   PA2
IO0（SI）   ->   PB1
IO1（SO）   ->   PB0
IO2（WP）   ->   PA7
IO3（HOLD） ->   PA6

P25Q64_2         MCU
CLK         ->   PB10
SS1（CS）   ->   PB11
IO4（SI）   ->   PC1
IO5（SO）   ->   PC2
IO6（WP）   ->   PC3
IO7（HOLD） ->   PC4

CS1 和 CS2 需接上拉电阻
IO2/IO3/IO6/IO7需接上拉电阻

Notes:
Wiring method:
P25Q64_ 1 MCU
CLK ->PB10
SS0 (CS) ->PA2
IO0 (SI) ->PB1
IO1 (SO) ->PB0
IO2 (WP) ->PA7
IO3 (HOLD) ->PA6


P25Q64_ 2 MCU
CLK ->PB10
SS1 (CS) ->PB11
IO4 (SI) ->PC1
IO5 (SO) ->PC2
IO6 (WP) ->PC3
IO7 (HOLD) ->PC4

CS1 and CS2 need to be connected with Pull-up resistor
IO2/IO3/IO6/IO7 needs to be connected with Pull-up resistor
================================================================================