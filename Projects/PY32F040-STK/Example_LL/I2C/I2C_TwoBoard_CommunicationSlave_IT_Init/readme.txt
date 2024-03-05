================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过中断方式进行I2C通讯。主机先向从机发送15字节的数据，然后再接收
从机发送的15字节的数据。当主机和从机成功接收数据后，主机和从机板上的小灯将保持
“常亮”状态。

Function descriptions:
This sample demonstrates communication between the master device and the slave 
device using I2C with interrupts. The master device first sends 15 bytes of 
data to the slave device and then receives 15 bytes of data from the slave 
device. Upon successful data transmission and reception, the LEDs on both the 
master and slave boards will remain continuously on.
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
3. 复位完成后，按下主机的用户按键，主机和从机开始通讯。
4. 观察主机和从机板上的LED灯，当LED灯由常暗状态转为常亮状态时，表示主机和从机
收发数据成功。如果LED灯处于闪烁状态，则表示主机和从机收发数据失败。

Example execution steps:
1. Compile and download the program to the MCU, and run it.
2. Reset the slave device first, and then reset the master device.
3. After the reset, press the user button on the master device to initiate 
   communication between the master and slave.
4. Observe the LEDs on both the master and slave boards. When the LEDs change 
   from off to on, it indicates that the master and slave devices have 
   successfully transmitted and received data. If the LEDs are blinking, it 
   indicates a data transmission failure.
================================================================================
注意事项：
PA9     ------> I2C_SCL
PA10    ------> I2C_SDA 
主机程序为I2C_TwoBoard_CommunicationMaster_IT_Init
从机程序为I2C_TwoBoard_CommunicationSlave_IT_Init
如需修改速率，直接修改I2C_SPEEDCLOCK即可。

Notes:
PA9     ------> I2C_SCL
PA10    ------> I2C_SDA 
Master program: I2C_TwoBoard_CommunicationMaster_IT_Init
Slave program: I2C_TwoBoard_CommunicationSlave_IT_Init
To modify the speed, simply change the I2C_SPEEDCLOCK value.
================================================================================
