================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述:
此样例是对串口外设接口（SPI）与外部设备以全双工串行方式进行通信的演示,此接口设
置为主模式，为外部从设备提供通信时钟SCK。主机通过MOSI引脚发送数据,从MISO引脚接收
从机的数据，数据以主机提供的SCK沿同步被移位，完成全双工通信。

Function descriptions:
This example demonstrates that SPI communicates with external devices in full 
duplex serial mode.This interface is set in master mode to provide communication 
clock SCK for external and slave devices.The host sends data through the MOSI 
pin, receives data from the MISO pin, and the data is shifted synchronously 
along the SCK provided by the host, completing the full-duplex communication.
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
2.编译下载主机程序（本样例程序）
3.编译下载从机程序SPI_TwoBoards_FullDuplexSlave_DMA
4.主机与从机引脚连接(箭头指向为信号传输方向) 
主机MASTER：         从机SLAVE：
SCK(PB3)   ----->    sck(PB3)
MISO(PB4)  <-----    MISO(PB4)
MOSI(PB5)  ----->    MOSI(PB5)
NSS(PA15)  ----->    NSS(PA15)
5.主从机上电
6.按下从机复位按键先运行从机程序，再按下主机用户按键运行主机程序 
7.观察主从机的LED灯，当主机和从机LED灯由常暗转为常亮状态，则表明主机、从机收发数
据成功；当主机或从机LED灯处于闪烁状态，则表明主机、从机收发数据失败。

Example execution steps:
1. Select two PY32F403_STK boards, one as the host and the other as the slave
2. Compile and download the host program (this sample program)
3. Compile and download slave program: SPI_TwoBoards_FullDuplexSlave_DMA
4. The host is connected to the SLAVE pin (the arrow points in the signal 
transmission direction).
MASTER：             SLAVE：
SCK(PB3)   ----->    sck(PB3)
MISO(PB4)  <-----    MISO(PB4)
MOSI(PB5)  ----->    MOSI(PB5)
NSS(PA15)  ----->    NSS(PA15)
5. Power on the host and slave
6. Press the slave reset button to run the slave program first, and then press 
the host user button to run the host program
7. Observe the LED lights of the master and slave. When the LED lights of the 
master and slave turn from normal dark to steady on, it indicates that the 
master and slave receive and send data successfully; When the LED of the host or 
slave is blinking, it indicates that the host or slave fails to receive and send 
data.
================================================================================
注意事项:
1.必须先按从机复位按键使从机程序先运行，再按主机用户按键开始运行主机程序，否则会
导致主从机通信失败。
2.主机模式可通过程序设置IO控制方式为DMA方式、中断方式和查询方式
3.NSS引脚只需要在使用NSS硬件方式时连接（本样例使用的是NSS硬件方式）

Note:
1. You must press the slave reset button to make the slave program run first, 
and then press the host user button to start running the host program, 
otherwise, the communication between the host and slave will be fail
2. The host mode can be programmed to set the IO control mode to DMA mode, 
interrupt mode and query mode
3.NSS pins need to be connected only when using NSS hardware mode (this example 
uses NSS hardware mode).
================================================================================
