================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例演示了ESMC的memory mapping功能，把预先编译好的bin文件，下载到P25Q64芯片中，
然后把P25Q64地址映射到0x00000000，主程序跳转到0x00000000地址开始执行bin文件中的
程序，样例中bin程序执行的任务是闪烁LED（PA01）灯。

Function descriptions:
This example demonstrates the memory mapping function of ESMC. The pre compiled
bin file is downloaded to the P25Q64 chip, and then the P25Q64 address is mapped
to 0x000000. The main program jumps to the 0x000000 address to start executing
the program in the bin file. In the example, the bin program executes the task
of flashing the LED (PA01) light.
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
P25Q64           MCU
CLK         ->   PB10
SS0（CS）   ->   PA2
IO0（SI）   ->   PB1
IO1（SO）   ->   PB0
IO2（WP）   ->   PA7
IO3（HOLD） ->   PA6

跳转后的程序的SystemCoreClock默认值需设置和当前程序的SystemCoreClock值一致。

Notes:
Wiring method:
P25Q64           MCU
CLK         ->   PB10
SS0 (CS)    ->   PA2
IO0 (SI)    ->   PB1
IO1 (SO)    ->   PB0
IO2 (WP)    ->   PA7
IO3 (HOLD)  ->   PA6

The default SystemCoreClock value of the program after the jump should be set 
to be consistent with the SystemCoreClock value of the current program.
================================================================================
