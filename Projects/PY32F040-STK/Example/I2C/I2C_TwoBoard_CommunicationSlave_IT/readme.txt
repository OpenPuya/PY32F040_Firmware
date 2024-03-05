================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了I2C通过中断方式进行通讯，主机先向从机发送15byte数据，然后再接收从机
发送的15byte数据，主机和从机接收数据成功后，主机和从机板上的小灯处于“常亮”状态。

Function descriptions:
This sample demonstrates the I2C communication using interrupt mode. The master 
sends 15 bytes of data to the slave, and then receives 15 bytes of data from the 
slave. When both the master and slave successfully receive the data, the LEDs on 
the master and slave boards will be constantly on.
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
3. 复位完成后，按下主机的用户按键，主机和从机开始通讯；
4. 观察主机和从机的LED灯，当主机和从机的LED灯由常暗转为常亮状态，则表示主机和从机
   收发数据成功；当主机或从机的LED灯处于闪烁状态，则表示主机和从机收发数据失败。

Example execution steps:
1. Compile and download the program to the MCU and run it.
2. Reset the slave first, and then reset the master.
3. After the reset is complete, press the user button on the master to start 
   communication between the master and slave.
4. Observe the LEDs on the master and slave boards. When the LEDs on both the 
   master and slave boards change from dim to constantly on, it indicates that 
   the master and slave have successfully transmitted and received data. If the 
   LEDs on the master or slave are blinking, it indicates a failure in data 
   transmission.
================================================================================
注意事项：
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA
主机程序为I2C_TwoBoard_CommunicationMaster_IT
从机程序为I2C_TwoBoard_CommunicationSlave_IT
如需修改速率，直接修改I2C_SPEEDCLOCK即可。

Notes:
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA
Master program: I2C_TwoBoard_CommunicationMaster_IT
Slave program: I2C_TwoBoard_CommunicationSlave_IT
To modify the speed, simply modify I2C_SPEEDCLOCK.
================================================================================
