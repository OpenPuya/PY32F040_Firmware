================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了通过中断方式，主机发送不定长数据，从机接收不定长数据。主机向从机发送
10字节的数据（0~9），然后从机接收数据（0~9）并通过串口打印；主机向从机发送100
字节数据（1~100），然后从机接收数据（1~100）并通过串口打印；主机向从机发送10
字节的数据（0~9），然后从机接收数据（0~9）并通过串口打印。

Function descriptions:
This example demonstrates how the host sends variable length data and the slave
receives variable length data through interrupt mode. The host sends 10 bytes of
data (0-9) to the slave, and then the slave receives the data (0-9) and prints
it through the serial port; The host sends 100 bytes of data (1-100) to the
slave, and then the slave receives the data (1-100) and prints it through the
serial port; The host sends 10 bytes of data (0-9) to the slave, and then the
slave receives the data (0-9) and prints it through the serial port.
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
4. 若从机通过串口先后打印0~9，1~100，0~9等3帧数据，则表示主机和从机通信成功；
   否则，主机和从机通信失败。

Example execution steps:
1. Compile and download the program to the MCU, and run it.
2. Reset the slave device first, and then reset the master device.
3. After the reset, press the user button on the master device to initiate 
   communication between the master and slave.
4. If the slave prints 3 frames of data through the serial port, including 0-9,
   1-100, 0-9, it indicates successful communication between the host and the
   slave; Otherwise, communication between the host and slave will fail.
================================================================================
注意事项：
（1）I2C引脚
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA
（2）串口连线
STK板        USB转TTL模块
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND       -->  GND
（3）主机程序和从机程序
主机程序为I2C_TwoBoard_MasterTxIndefiniteLengthData_IT_Init
从机程序为I2C_TwoBoard_SlaveRxIndefiniteLengthData_IT_Init
（4）更改通信速率
如需修改速率，直接修改I2C_SPEEDCLOCK即可。
（5）从机接收单帧数据长度限制
用户可根据需要修改从机程序main.c中宏RX_MAX_LEN的值，RX_MAX_LEN定义了从机单次
接收数据长度（当前样例为200byte）。

Notes:
(1) I2C pin
PB6  -------> I2C1_SCL
PB7  -------> I2C1_SDA
(2) UART connection
STK board    USB to TTL moudle
PA02(TX)  -->  RX
PA03(RX)  -->  TX
GND      -->  GND
(3) Master program and slave program
Master program: I2C_TwoBoard_MasterTxIndefiniteLengthData_IT_Init
Slave program: I2C_TwoBoard_SlaveRxIndefiniteLengthData_IT_Init
(4) Change communication rate
To modify the speed, simply change the I2C_SPEEDCLOCK value.
(5) Length limit for single frame data received by the slave
Users can modify the value of macro RX_MAX_LEN in the slave program main.c as
needed, which defines the length of data received by the slave in a single
attempt (currently 200bytes).
================================================================================
