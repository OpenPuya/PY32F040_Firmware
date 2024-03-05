================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了主机I2C通过DMA方式进行通讯，从机使用EEPROM外设芯片P24C32，按下user按
键，主机先向从机写15bytes数据为0x1~0xf，然后再从EEPROM中将写入的数据读出，读取成
功后，主机板上的小灯处于“常亮”状态。

Function descriptions:
This sample demonstrates communication between the master and slave using I2C 
and DMA. The master device sends 15 bytes of data (0x1~0xf) to the slave device, 
and then reads the data from the EEPROM. When the data is successfully read, 
the LED on the master board will be constantly lit.
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
2. 先复位从机，然后再复位主机；
3. 复位完成后，按下主机user按键，先写15bytes数据0x1-0xf；
4. 延时100ms后，从eeprom读取15bytes数据；
5. 观察主机的LED灯，当主机LED灯由常暗转为常亮状态，则表明主机收发数据成功；当
   主机LED灯处于闪烁状态，则表明主机收发数据失败。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Reset the slave first, and then reset the master.
3. After the reset is complete, press the user button on the master. The master 
   will write 15 bytes of data (0x1-0xf).
4. After a delay of 100ms, the master will read 15 bytes of data from the EEPROM.
5. Observe the LED on the master board. When the LED changes from off to constantly 
   lit, it indicates successful data transmission and reception. If the LED is 
   blinking, it indicates data transmission or reception failure.
================================================================================
注意事项：
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA

Notes:
PB6 -------> I2C1_SCL
PB7 -------> I2C1_SDA
================================================================================
