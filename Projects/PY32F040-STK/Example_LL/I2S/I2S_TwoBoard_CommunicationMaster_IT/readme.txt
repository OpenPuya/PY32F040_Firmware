================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例是对I2S主机与I2S从机以中断方式进行通信的演示，I2S主机先向I2S从机发送数
据0x1~0x10，I2S从机接收到数据后，再向I2S主机回发数据0x0x1~0x10，当I2S主机、I2S
从机成功接收数据时，小灯处于常亮状态，否则小灯处于闪烁状态。

Function descriptions:
This sample demonstrates communication between an I2S master and an I2S slave 
using interrupt mode. The I2S master sends data 0x1 to 0x10 to the I2S slave. 
After receiving the data, the I2S slave sends data 0x1 to 0x10 back to the I2S 
master. When the I2S master and slave successfully receive the data, the LED 
remains lit. Otherwise, the LED blinks.
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
1.选择两块PY32F040_STK板，一块作为主机，一块作为从机
2.编译下载主机程序I2S_TwoBoard_CommunicationMaster_IT
3.编译下载从机程序I2S_TwoBoard_CommunicationSlave_IT
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
1. Select two PY32F040_STK boards, one as the master and the other as the slave.
2. Compile and download the master program, I2S_TwoBoard_CommunicationMaster_IT.
3. Compile and download the slave program, I2S_TwoBoard_CommunicationSlave_IT.
4. Connect the master and slave pins (arrow indicates the signal direction):
   Master (MASTER):             Slave (SLAVE):
   SD (PB15)    <----->         SD (PB15)
   WS (PB12)    ----->          WS (PB12)
   CK (PB13)    ----->          CK (PB13)
   GND          <----->         GND
5. Power on both the master and slave, and press the reset button on each.
6. Press the user button on the slave board to run the slave program first, 
   then press the user button on the master board to run the master program.
7. Observe the LED lights on the master and slave boards. If the LED lights 
   on both the master and slave boards turn from dim to solid, it indicates 
   successful data transmission between the master and slave. If the LED light 
   on the master or slave board is blinking, it indicates a failure in data 
   transmission.
================================================================================
注意事项：

Notes:
================================================================================
