#! /usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
from std_msgs.msg import String
import struct
import numpy as np
import time

"""
    使用python实现消息发布：
        1.导包
        2.初始化ROS
        3.创建发布者对象
        4.变现发布逻辑并发布数据

"""

def hextofloat(h):
    
    return struct.unpack("<f",h)[0]

if __name__ == "__main__":
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    # 方式1：调用函数接口打开串口时传入配置参数

 
    ser = serial.Serial("/dev/ttyUSB0", 460800)    # 打开COM17，将波特率配置为115200，其余参数使用默认值
    if ser.isOpen():                        # 判断串口是否成功打开
        print("打开串口成功。")
        print(ser.name)    # 输出串口号
    else:
        print("打开串口失败。")
    # rospy.init_node("demo01_pub_p")
    # rate = rospy.Rate(1)
    data = [0x49,0xAA,0x0D,0x0A]

    while True:
        write_len = ser.write(data)
        print("串口发出{}个字节。".format(write_len))
        com_input = ser.read(28)
        input = []
        for i in range (28):
            input.append(hex(com_input[i]))

        if com_input:   # 如果读取结果非空，则输出
            if input[0] == '0x49' and input[1] == '0xaa':
                print('fx:',hextofloat(com_input[2:6]),end="\t")
                print('fy:',hextofloat(com_input[6:10]),end="\t")
                print('fz:',hextofloat(com_input[10:14]),end="\t")
                print('Mx:',hextofloat(com_input[14:18]),end="\t")
                print('My:',hextofloat(com_input[18:22]),end="\t")
                print('Mz:',hextofloat(com_input[22:26]),end="\t")
        # rate.sleep()
        time.sleep(0.05)
    
    ser.close()


    # rospy.init_node("demo01_pub_p")
    # pub = rospy.Publisher("che",String,queue_size=10)
    # # 创建数据
    # msg = String()
    # # 制定发布频率
    # rate = rospy.Rate(1) # 以秒为单位计时
    # count = 0
    # rospy.sleep(3)
    # while not rospy.is_shutdown():
    #     count +=1
    #     msg.data = "hello" + str(count)
    #     pub.publish(msg)

    #     rate.sleep()