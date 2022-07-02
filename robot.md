# 主要模块关系
电机模块，电机控制模块，通信模块
## 电机模块
Motor类->SerialComMotor类(添加通信模块AsyncSerialDevice类的实例为成员)->pan_tilt_motor类
## 电机控制模块
SerialDeviceManager类->PanTiltMotorManager类
## 通信模块
CommunicationDevice类->AsyncListenerInterface类->AsyncSerialDevice类
# 电机模块详解
## Motor类
抽象类，定义了电机name，id和enable上电使能，定义了电机init()的接口。
## SerialComMotor类
使用串口通信的电机的类，继承自Motor类，比Motor类多了一个异步串口通信设备(AsyncSerialDevice类)的成员。
## pan_tilt_motor类
实际使用的电机对应的类，pan，tilt分别指水平和垂直旋转的电机。

继承自SerialComMotor类，定义了电机实时状态PanTiltMotorStateFeedback，电机配置参数PanTiltMotorConfig与配置参数的互斥锁，电机控制命令PanTiltMotorControlCmd与控制命令的互斥锁。

pan_tilt_motor类中定义并实现了所有电机控制方法，每种控制都是通过配置对应的电机控制命令帧PanTiltMotorControlCmd，然后通过异步串口通信设备AsyncSerialDevice类的SendData方法发送控制命令。这些电机控制方法将会在电机控制模块控制电机时发生调用。
# 电机控制模块详解















