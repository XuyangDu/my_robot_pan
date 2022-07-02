把"usbcan.ko、libusbcan.so.1"拷到"/lib"目录下，创建符号链接。
# ln -s /lib/libusbcan.so.1 /lib/libusbcan.so"

驱动的加载：
# insmod /lib/usbcan.ko

查看驱动是否加载成功，成功则会列出一行模块信息，如果没有表示加载失败：
# lsmod | grep usbcan


切换目录到"./test"。
编译测试程序"make"－注：此步骤可选，"./test"目录下已经有编译的好的。
运行测试程序"./test"。

test会对每个通道以自发自收方式进行测试，如果板卡正常，会在结束时显示收发帧数和发送速度。


ubuntu12.04 kernel 3.2.0-30-generic