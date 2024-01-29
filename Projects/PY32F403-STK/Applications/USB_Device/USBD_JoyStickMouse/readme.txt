================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了USB鼠标功能。
USB鼠标主要通过USB HID类实现，将其模拟成一个USB鼠标设备。设备程序每隔1秒钟
向主机发送一次鼠标相对位置。在PC端，您可以看到鼠标每隔1秒移动一次位置。

Function descriptions:
This sample demonstrates the USB mouse functionality.
The USB mouse is mainly implemented using the USB HID class, simulating a USB 
mouse device.The device program sends the mouse relative position to the host 
every 1 second.On the PC side, you can observe the mouse moving its position 
every 1 second.

================================================================================
测试环境：
测试用板：STK
MDK版本：5.28

Test environment:
Test board: STK
MDK Version: 5.28

================================================================================
使用步骤：
1. 下载并运行程序；
2. 通过USB连接线，一端连接STK板，另一端连接PC电脑；
3. PC端鼠标每隔1秒钟移动一次位置；

Example execution steps:
1. Download and run the program;
2. Connect the STK board to the PC computer using the USB cable;
3. The mouse position will be updated every 1 second on the PC side.

================================================================================
注意事项：
1. 用户测试程序过程中，如果对USB接口进行热插拔，当重新插上USB线到PC端后，
   有可能鼠标不再移动。这是因为程序每次发送新坐标位置之前会判断上一次是否发送
   成功（检查标志位HID_STATE_BUSY）。如果热插拔过程中导致USB通讯失败一次，则
   会停止更新鼠标位置。您可以通过在程序中取消对HID_STATE_BUSY标志位的判断，即
   可实现热插拔不影响鼠标移动。

Notes:
1. During user testing, if the USB interface is hot-plugged, and after 
   re-plugging the USB cable to the PC, the mouse movement may stop. 
   This is because before sending a new mouse coordinate, the program 
   checks if the previous transmission was successful (check the 
   HID_STATE_BUSY flag). If there is a USB communication failure 
   during the hot-plug process, it will stop updating the mouse 
   position. You can implement hot-plug without affecting the mouse 
   movement by removing the check for the HID_STATE_BUSY flag in 
   the program.

================================================================================
