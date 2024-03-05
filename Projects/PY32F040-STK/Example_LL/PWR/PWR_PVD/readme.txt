================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了PVD（电压检测功能）。样例中配置PB07引脚的电压与VREF（1.2V）进行比较。
当PB07引脚的电压高于VREF时，LED灯灭；当低于VREF时，LED灯亮。

Function descriptions:
This sample demonstrates the PVD (Voltage Detection) function. The voltage at 
PB07 pin is compared with VREF (1.2V). When the voltage at PB07 is higher than 
VREF, the LED light is off. When the voltage is lower than VREF, the LED light 
is on.
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
2. 当PB07引脚输入0V时，进入PVD中断，LED灯亮；
   当PB07引脚悬空时（内部上拉相当于接入3.3V），LED灯灭；
3. 再次输入0V，LED灯亮。

Example execution steps:
1. Compile and download the program to the MCU.
2. When the voltage at PB07 pin is 0V, it triggers the PVD interrupt and the 
   LED light turns on. When the PB07 pin is left floating (internally pulled 
   up to 3.3V), the LED light turns off.
3. Input 0V again, and the LED light turns on.
================================================================================
注意事项：

Notes:
================================================================================
