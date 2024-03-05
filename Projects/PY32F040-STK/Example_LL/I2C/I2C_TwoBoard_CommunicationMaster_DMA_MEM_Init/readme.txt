================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了主机I2C通过DMA方式进行通讯，从机使用EEPROM外设芯片P24C32。按下用户
按键后，主机先向从机写入15字节的数据（0x1-0xf），然后从EEPROM中读取写入的数据。
读取成功后，主机板上的小灯处于“常亮”状态。

Function descriptions:
This sample demonstrates communication between the master device and the slave 
device using I2C with DMA. The slave device uses the EEPROM peripheral chip 
P24C32. When the user button is pressed, the master device first writes 15 
bytes of data (0x1-0xf) to the slave device, and then reads the written data 
from the EEPROM. Upon successful read, the LED on the master board remains 
continuously on.
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
1. 编译下载程序到MCU，并运行。
2. 先复位从机，然后再复位主机。
3. 复位完成后，按下主机的用户按键，主机先向从机写入15个字节的数据（0x1-0xf）。
4. 等待100毫秒后，从EEPROM中读取15个字节的数据。
5. 观察主机板上的LED灯，当LED灯由常暗状态转为常亮状态时，表示主机收发数据成功。
   如果LED灯处于闪烁状态，则表示主机收发数据失败。

Example execution steps:
1. Compile and download the program to the MCU, and run it.
2. Reset the slave device first, and then reset the master device.
3. After the reset, press the user button on the master device. The master 
   device will write 15 bytes of data (0x1-0xf) to the slave device.
4. Wait for 100 milliseconds, and then read 15 bytes of data from the EEPROM.
5. Observe the LED on the master board. When the LED changes from off to on, 
   it indicates that the master device has successfully transmitted and received 
   data. If the LED is blinking, it indicates a data transmission failure.
================================================================================
注意事项：
PA9     ------> I2C_SCL
PA10    ------> I2C_SDA

Notes:
PA9     ------> I2C_SCL
PA10    ------> I2C_SDA================================================================================
