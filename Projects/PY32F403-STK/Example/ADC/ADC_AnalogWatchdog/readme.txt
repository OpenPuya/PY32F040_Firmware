================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了ADC的模拟看门狗功能，当开启看门狗的通道的电压值不在设定的上下限中，
会进入看门狗中断。

Function descriptions:
This example demonstrates the analog watchdog function of ADC. When the voltage
value of the channel that opens the watchdog is not within the set upper or 
lower limits,Will enter watchdog interrupt.
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
使用步骤：
1. 编译下载程序到MCU，并运行；
2. PA4的电压值小于1.65V(供电电压为3.3V),不会进入看门狗中断，LED灯不亮；
3. PA4的电压值大于1.65V(供电电压为3.3V)，进入看门狗中断，LED亮。

Example execution steps:
1. Compile and download the program to MCU and run it;
2. If the voltage value of PA4 is less than 1.65V (with a power supply voltage of
3.3V), it will not enter the watchdog interrupt and the LED light will not light 
up;
3. If the voltage value of PA4 is greater than 1.65V (with a supply voltage of 
3.3V), the watchdog will interrupt and the LED will light up.
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

STK board     USB to TTL module
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND
================================================================================