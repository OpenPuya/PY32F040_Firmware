================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过DMA方式进行I2S主机与I2S从机的通信。I2S主机先向I2S从机发送数据
0x1~0x10，I2S从机接收到数据后，再向I2S主机回发数据0x1~0x10。当I2S主机和I2S从机
成功接收数据时，LED灯将保持常亮状态；否则LED灯将处于闪烁状态。

Function descriptions:
This sample demonstrates communication between an I2S master and an I2S slave 
using DMA. The I2S master device first sends data from 0x1 to 0x10 to the I2S 
slave device. The I2S slave device receives the data and then sends data from 
0x1 to 0x10 back to the I2S master device. When both the I2S master and I2S 
slave successfully receive the data, the LED lights remain continuously on. 
Otherwise, the LED lights will blink.
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
1. 准备两块PY32F040_STK开发板，一块作为主机，一块作为从机。
2. 编译并下载主机程序I2S_TwoBoard_CommunicationMaster_DMA。
3. 编译并下载从机程序I2S_TwoBoard_CommunicationSlave_DMA。
4. 连接主机与从机的引脚（箭头指向为信号传输方向）：
   主机MASTER：          从机SLAVE：
   SD(PB15)    <----->   SD(PB15)
   WS(PB12)    ----->    WS(PB12)
   CK(PB13)    ----->    CK(PB13)
   GND         <----->   GND
5. 上电主机和从机，并按下复位按钮复位。
6. 先按下从机上的用户按键运行从机程序，然后按下主机上的用户按键运行主机程序。
7. 观察主机和从机板上的LED灯，当LED灯由常暗状态转为常亮状态时，表示主机和从机
   收发数据成功。如果LED灯处于闪烁状态，则表示主机和从机收发数据失败。

Example execution steps:
1. Prepare two PY32F040_STK development boards, one as the master and one as 
   the slave.
2. Compile and download the master program "I2S_TwoBoard_CommunicationMaster_DMA".
3. Compile and download the slave program "I2S_TwoBoard_CommunicationSlave_DMA".
4. Connect the pins between the master and slave devices (arrow indicates signal 
   direction):
   Master (MASTER):       Slave (SLAVE):
   SD (PB15)    <----->   SD (PB15)
   WS (PB12)    ----->    WS (PB12)
   CK (PB13)    ----->    CK (PB13)
   GND          <----->   GND
5. Power on both the master and slave devices, and press the reset button.
6. Press the user button on the slave device to run the slave program first, 
   and then press the user button on the master device to run the master program.
7. Observe the LED lights on both the master and slave boards. When the LED 
   lights change from off to on, it indicates that the master and slave devices 
   have successfully transmitted and received data. If the LED lights are 
   blinking, it indicates a data transmission failure.
================================================================================
注意事项：

Notes:
================================================================================
