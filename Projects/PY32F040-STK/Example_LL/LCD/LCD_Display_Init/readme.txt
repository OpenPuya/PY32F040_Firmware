================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例是对单色无源液晶显示器(LCD)的演示，将偏置产生电路配置为内部电阻分压，
使LCD全显，显示“88:88”字样。

Function descriptions:
This sample demonstrates the usage of a monochrome passive LCD (Liquid Crystal 
Display). The bias generation circuit is configured with internal resistor 
voltage division to achieve a fully displayed LCD showing the "88:88" characters.
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
1. 编译并下载程序；
2. STK板与LCD的连线方式如下：
   STK板                LCD
   PA9(COM0)  --------> COM0
   PA10(COM1) --------> COM1
   PA11(COM2) --------> COM2
   PA12(COM3) --------> COM3
   PA8(SEG0)  --------> SEG0
   PC9(SEG1)  --------> SEG1
   PC8(SEG2)  --------> SEG2
   PC7(SEG3)  --------> SEG3
   PC6(SEG4)  --------> SEG4
   PB15(SEG5) --------> SEG5
   PB14(SEG6) --------> SEG6
   PB13(SEG7) --------> SEG7
3. 按下复位按键，LCD全显，显示“88:88”字样。

Example execution steps:
1. Compile and download the program.
2. Connect the STK board to the LCD using the following connections:
   STK board                LCD
   PA9(COM0)   --------> COM0
   PA10(COM1)  --------> COM1
   PA11(COM2)  --------> COM2
   PA12(COM3)  --------> COM3
   PA8(SEG0)   --------> SEG0
   PC9(SEG1)   --------> SEG1
   PC8(SEG2)   --------> SEG2
   PC7(SEG3)   --------> SEG3
   PC6(SEG4)   --------> SEG4
   PB15(SEG5)  --------> SEG5
   PB14(SEG6)  --------> SEG6
   PB13(SEG7)  --------> SEG7
3. Press the reset button to display the "88:88" characters on the LCD.
================================================================================
注意事项：
单色无源液晶显示器(LCD)的型号为：GDC04212。

Notes:
The model of the monochrome passive LCD is: GDC04212.
================================================================================
