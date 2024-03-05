================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了I2C通过中断方式进行通讯，主机先向从机发送15byte数据，然后再接收从机
发送的15byte数据,主机、从机接收数据成功后，主机和从机板上的小灯处于“常亮”状态。

Function descriptions:
This sample demonstrates I2C communication using interrupts. The master device 
sends 15 bytes of data to the slave device and then receives 15 bytes of data 
from the slave. When both the master and slave successfully transmit and 
receive data, the LEDs on both boards will be constantly lit.
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
2. 先复位从机，然后再复位主机
3. 复位完成后，按下主机user按键，主从开始通讯
4. 观察主从机的LED灯，当主机和从机LED灯由常暗转为常亮状态，则表明主机、从机收发数
   据成功；当主机或从机LED灯处于闪烁状态，则表明主机、从机收发数据失败。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Reset the slave first, and then reset the master.
3. After the reset is complete, press the user button on the master to initiate 
   communication between the master and slave.
4. Observe the LEDs on the master and slave boards. When both LEDs are constantly 
   lit, it indicates successful data transmission and reception between the master 
   and slave. If any of the LEDs are blinking, it indicates data transmission or 
   reception failure.
================================================================================
注意事项：
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA
主机程序为I2C_TwoBoard_CommunicationMaster_IT
从机程序为I2C_TwoBoard_CommunicationSlave_IT
如需修改速率，直接修改I2C_SPEEDCLOCK即可。

Notes:
PB6 -------> I2C1_SCL
PB7 -------> I2C1_SDA
Master program: I2C_TwoBoard_CommunicationMaster_IT
Slave program: I2C_TwoBoard_CommunicationSlave_IT
To change the communication speed, modify I2C_SPEEDCLOCK directly.
================================================================================