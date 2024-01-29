================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例是对I2S主机与I2S从机以DMA方式进行通信的演示，I2S主机先向I2S从机发送数
据0x1~0x10，I2S从机接收到数据后，再向I2S主机回发数据0x0x1~0x10，当I2S主机、I2S
从机成功接收数据时，小灯处于常亮状态，否则小灯处于闪烁状态。

Function descriptions:
This sample demonstrates communication between the I2S master and I2S slave 
using DMA. The I2S master sends data 0x1 to 0x10 to the I2S slave. The I2S 
slave receives the data and sends back data 0x1 to 0x10 to the I2S master. 
When both the I2S master and I2S slave successfully receive the data, the 
LED will be constantly on. Otherwise, the LED will be blinking.
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
使用步骤:
1.选择两块PY32F403_STK板，一块作为主机，一块作为从机
2.编译下载主机程序I2S_TwoBoard_CommunicationMaster_DMA
3.编译下载从机程序I2S_TwoBoard_CommunicationSlave_DMA
4.主机与从机引脚连接(箭头指向为信号传输方向) 
主机MASTER：         从机SLAVE：
SD(PB15)    <----->   SD(PB15)
WS(PB12)    ----->    WS(PB12)
CK(PB13)    ----->    CK(PB13)
GND         <----->   GND
5.主、从机上电后，并按下主、从机复位按键
6.按下从机用户按键先运行从机程序，再按下主机用户按键运行主机程序 
7.观察主从机的LED灯，当主机和从机LED灯由常暗转为常亮状态，则表明主机、从机收发数
据成功；当主机或从机LED灯处于闪烁状态，则表明主机、从机收发数据失败。

Example execution steps:
1. Select two PY32F403_STK boards, one as the master and one as the slave.
2. Compile and download the master program I2S_TwoBoard_CommunicationMaster_DMA.
3. Compile and download the slave program I2S_TwoBoard_CommunicationSlave_DMA.
4. Connect the pins of the master and slave boards (direction indicated by the 
   arrow):
   Master MASTER:         Slave SLAVE:
   SD(PB15)    <----->   SD(PB15)
   WS(PB12)    ----->    WS(PB12)
   CK(PB13)    ----->    CK(PB13)
   GND         <----->   GND
5. Power on the master and slave boards, and press the reset button on both.
6. Press the user button on the slave first to run the slave program, then press 
   the user button on the master to run the master program.
7. Observe the LEDs on the master and slave boards. When the LEDs on both the 
   master and slave boards change from dim to constantly on, it indicates that 
   the master and slave have successfully transmitted and received data. If the 
   LEDs on the master or slave are blinking, it indicates a failure in data 
   transmission.
================================================================================
注意事项:

Notes:
================================================================================
