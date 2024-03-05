================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例是对I2S主机与I2S从机以DMA方式进行通信的演示，I2S主机先向I2S从机发送数据
0x1~0x10，I2S从机接收到数据后，再向I2S主机回发数据0x1~0x10。当I2S主机和I2S从机
成功接收数据时，小灯处于常亮状态；否则，小灯处于闪烁状态。

Function descriptions:
This sample demonstrates communication between an I2S master and an I2S slave 
using DMA. The I2S master sends data 0x1 to 0x10 to the I2S slave, and the I2S 
slave receives the data and sends back data 0x1 to 0x10 to the I2S master. When 
both the I2S master and I2S slave successfully receive the data, the LED remains 
constantly on. Otherwise, the LED blinks.
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
1. 准备两块PY32F040_STK板，一块作为主机，一块作为从机。
2. 编译下载主机程序 I2S_TwoBoard_CommunicationMaster_DMA。
3. 编译下载从机程序 I2S_TwoBoard_CommunicationSlave_DMA。
4. 连接主机和从机的引脚（箭头指向为信号传输方向）：
   主机 MASTER:        从机 SLAVE:
   SD (PB15)    <----->   SD (PB15)
   WS (PB12)    ----->    WS (PB12)
   CK (PB13)    ----->    CK (PB13)
   GND          <----->   GND
5. 上电主机和从机，并按下主机和从机的复位按键。
6. 先按下从机用户按键，运行从机程序，然后再按下主机用户按键，运行主机程序。
7. 观察主机和从机的LED灯状态。当主机和从机的LED灯由常暗状态转为常亮状态时，表示
   主机和从机成功收发数据；如果主机或从机的LED灯闪烁，则表示主机和从机收发数据
   失败。

Example execution steps:
1. Prepare two PY32F040_STK boards, one as the master and the other as the slave.
2. Compile and download the master program, I2S_TwoBoard_CommunicationMaster_DMA.
3. Compile and download the slave program, I2S_TwoBoard_CommunicationSlave_DMA.
4. Connect the pins between the master and slave (arrow indicates the signal 
   direction):
   Master (MASTER):     Slave (SLAVE):
   SD (PB15)    <----->   SD (PB15)
   WS (PB12)    ----->    WS (PB12)
   CK (PB13)    ----->    CK (PB13)
   GND          <----->   GND
5. Power on the master and slave boards, and press the reset buttons on both.
6. Press the user button on the slave board to run the slave program, then press 
   the user button on the master board to run the master program.
7. Observe the LED status on the master and slave boards. When both LEDs change 
   from dim to constant brightness, it indicates successful data communication 
   between the master and slave. If the LEDs on the master or slave board are 
   blinking, it indicates a failure in data communication.
================================================================================
注意事项：

Notes:
================================================================================
