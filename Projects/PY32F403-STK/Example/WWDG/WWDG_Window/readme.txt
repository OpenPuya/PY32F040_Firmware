================================================================================
                                样例使用说明
                                Sample Description
================================================================================
功能描述：
此样例演示了WWDG的 窗口看门狗功能，配置WWDG的窗口上限（下限固定是0x3F），程序中
通过delay延时函数，确保程序是在WWDG计数窗口内进行喂狗动作，通过LED灯闪烁，可以判
断窗口内喂狗并未产生复位。

Function descriptions:
This example demonstrates the window watchdog function of WWDG. Set the upper
limit of the window of WWDG (the lower limit is fixed at 0x3F).The program
ensures that the WWDG is refreshed in the WWDG counting window through the delay
function, and can judge that the WWDG is refreshed in the window without
resetting through the LED light blinking.
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
1. 编译下载程序并运行；
2. 程序默认会在窗口时间内喂狗，程序不会复位，LED灯一直闪烁；
3. 注释掉宏定义”#WINDOW_IN",打开宏定义“WINDOW_UPPER”，重新编译下载并运行；
4. 程序会一直复位,LED灯循环点亮4s，熄灭500ms；
5. 注释掉宏定义”#WINDOW_UPPER",打开宏定义“WINDOW_LOWER”,重新编译下载并运行；
6. 程序会一直复位,LED灯循环点亮4s，熄灭500ms。

Example execution steps:
1. compile and download the program to MCU and run it.
2. By default, the program will refresh WWDG within the window time without 
resetting, and the LED light will keep blinking;
3. Mask the macro definition "#WINDOW_IN",unblock the macro definition
" WINDOW_UPPER ", re-compile, download and run;
4. The program will always reset,LED light cycle on 4s, off 500ms;
5. Mask the macro definition "#WINDOW_UPPER", unblock the macro definition
" WINDOW_LOWER ", recompile, download and run; 
6. The program will always reset,LED light cycle on 4s, off 500ms.
================================================================================
注意事项：

Notes:

================================================================================