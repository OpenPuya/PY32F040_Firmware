================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了IWDG看门狗功能。配置看门狗的重载计数值为1秒，当计数达到1秒后，系统
会被复位。通过调整每次喂狗的时间（main函数while循环中的代码），可以观察到以下情
况：如果每次喂狗时间小于1秒，程序能够正常运行（LED灯闪烁）；如果喂狗时间超过1秒，
程序会一直被复位（LED灯熄灭）。

Function descriptions:
This sample demonstrates the IWDG (Independent Watchdog) functionality. The 
watchdog is configured with a reload value of 1 second. Once the watchdog 
timer reaches 1 second, the system will be reset. By adjusting the time for 
feeding the watchdog (code in the main loop), the following observations can 
be made:If the feeding time is less than 1 second, the program can run 
normally (LED blinks);If the feeding time exceeds 1 second, the program will 
be continuously reset (LED turns off).
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
1. 编译并下载程序到MCU，然后运行；
2. 观察LED是否能正常亮起；
3. 修改程序（在main函数的while循环中注释的代码处），将喂狗时间设置为1.1秒；
4. 重新编译并下载程序到MCU，然后运行；
5. 观察LED是否能正常亮起。

Example execution steps:
1. Compile and download the program to the MCU, and then run it.
2. Observe if the LED lights up properly.
3. Modify the program (at the commented code in the main loop) to set the 
   feeding time to 1.1 seconds.
4. Re-compile and download the program to the MCU, and then run it.
5. Observe if the LED lights up properly.
================================================================================
注意事项：

Notes:
================================================================================
