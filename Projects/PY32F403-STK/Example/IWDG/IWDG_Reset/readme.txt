================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了IWDG看门狗功能，配置看门狗重载计数值，计数800ms后复位，然后通过调整
每次喂狗的时间（main函数while循环中代码），可以观察到，如果每次喂狗时间750ms，程
序能一直正常运行（LED灯闪烁），如果喂狗时间850ms，程序会一直复位（LED灯熄灭）。

Function descriptions:
This example demonstrates the function of IWDG (Independent Watchdog).Set IWDG 
to count 800ms and then reset.By adjusting the time of refresh the dog
each time (code in the main function while loop), it can be observed that if the
time is 750ms, the program can always run normally (LED blink), if the time is 
850ms, the program will always reset (LED off). 
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
2. 可观察到LED能正常闪烁；
3. 修改程序（main函数while循环中注释代码），配置喂狗时间为850ms；
4. 重新编译下载程序到MCU，并运行；
5. 可观察到LED不能正常亮起。

Example execution steps:
1. Compile and download the program to the MCU, and then run it.
2. Observe the LED blinking
3. Modify the program (at the commented code in the main loop) to set the 
   feeding time to 850 milliseconds.
4. Re-compile and download the program to the MCU, and then run it.
5. Observe the LED off.
================================================================================
注意事项：

Notes:

================================================================================