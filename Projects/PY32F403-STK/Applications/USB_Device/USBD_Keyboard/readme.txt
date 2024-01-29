================================================================================
                                样例使用说明
                             Sample Description
================================================================================
功能描述：
此样例演示了USB键盘功能。
USB键盘主要通过USB HID类实现，将其模拟成一个USB键盘设备。设备程序运行后，
会不断向主机发送字母a。在PC端，打开一个文本文档，可以看到文本中会自动持续的输入
字母a。

Function descriptions:
This sample demonstrates the USB keyboard functionality.
The USB keyboard is mainly implemented using the USB HID class, simulating a 
USB keyboard device. After the device program is running, it will continuously 
send the letter "a" to the host. On the PC side, open a text document, switch 
the input method to English input, and select the input window of the document 
with the mouse. You will see that the letter "a" is automatically typed 
continuously in the text document.

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
2. 在PC电脑上打开一个文本文档，输入法改成英文输入，并用鼠标选中文档输入窗口；
3. 通过USB连接线，一端连接STK板，另一端连接PC电脑；
4. 文本文档中，会持续输入字母a；

Example execution steps:
1. Download and run the program;
2. On the PC computer, open a text document, switch the input method to 
   English input, and select the input window of the document with the mouse;
3. Connect the STK board to the PC computer using the USB cable;
4. The text document will continuously input the letter "a".

================================================================================
注意事项：
1. 用户测试程序过程中，如果对USB接口进行热插拔，当重新插上USB线到PC端后，有可能
   上位机不会再收到数据。这是因为程序每次发送新数据之前会判断上一次是否发送成功
   （检查标志位HID_STATE_BUSY）。如果热插拔过程中导致USB通讯失败一次，则会停止
   发送数据。您可以通过在程序中取消对HID_STATE_BUSY标志位的判断，即可实现热插拔
   不影响数据发送。

Notes:
1. During user testing, if the USB interface is hot-plugged, and after 
   re-plugging the USB cable to the PC, the host may not receive data. This 
   is because before sending new data, the program checks if the previous 
   transmission was successful (check the HID_STATE_BUSY flag). If there is 
   a USB communication failure during the hot-plug process, it will stop 
   sending data. You can implement hot-plug without affecting data transmission 
   by removing the check for the HID_STATE_BUSY flag in the program.

================================================================================
