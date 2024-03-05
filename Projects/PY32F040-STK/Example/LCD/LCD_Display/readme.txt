================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了对单色无源液晶显示器（LCD）的操作。将偏置产生电路配置为内部电阻分压，
以使LCD全显，并显示“88:88”字样。

Function descriptions:
This sample demonstrates the operation of a monochrome passive liquid crystal 
display (LCD). The biasing circuit is configured with internal resistor division 
to ensure full display on the LCD, showing the text "88:88".
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
1. 编译并下载程序。
2. 连接STK板与LCD的引脚如下：
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
3. 按下复位按键，LCD将全显，显示“88:88”字样。

Example execution steps:
1. Compile and download the program.
2. Connect the pins of the STK board and the LCD as follows:
   STK board           LCD
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
3. Press the reset button, and the LCD will display "88:88" in full.
================================================================================
注意事项：
单色无源液晶显示器（LCD）的型号为GDC04212。

Notes:
The model of the monochrome passive liquid crystal display (LCD) is GDC04212.
================================================================================
