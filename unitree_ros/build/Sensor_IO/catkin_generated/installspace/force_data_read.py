#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports
from geometry_msgs.msg import Wrench, Vector3
import struct
import time

# 将16进制数据转换为浮点数
def hextofloat(h):
    return struct.unpack("<f", bytes(h))[0]

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("serial_to_ros_publisher")

    # 获取串口设备列表
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    # 配置并打开串口
    ser = serial.Serial("/dev/ttyUSB0", 460800)  # 修改为实际串口
    if ser.isOpen():
        print("打开串口成功：", ser.name)
    else:
        print("打开串口失败。")
        exit()

    # 创建发布者，发布的数据类型为Wrench
    pub = rospy.Publisher("wrench", Wrench, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(10)  # 每秒10次发布

    while not rospy.is_shutdown():
        # 发送数据给串口设备
        data = [0x49, 0xAA, 0x0D, 0x0A]
        write_len = ser.write(data)
        print("串口发出{}个字节".format(write_len))

        # 从串口读取28个字节的数据
        com_input = ser.read(28)
        input_data = [hex(byte) for byte in com_input]

        # 如果读取到数据并且是有效的（以0x49和0xAA开始）
        if com_input and input_data[0] == '0x49' and input_data[1] == '0xaa':
            # 解析数据，提取力和力矩信息
            fx = hextofloat(com_input[2:6])
            fy = hextofloat(com_input[6:10])
            fz = hextofloat(com_input[10:14])
            mx = hextofloat(com_input[14:18])
            my = hextofloat(com_input[18:22])
            mz = hextofloat(com_input[22:26])

            # 输出数据
            rospy.loginfo("fx: %f, fy: %f, fz: %f, Mx: %f, My: %f, Mz: %f",
                          fx, fy, fz, mx, my, mz)

            # 创建Wrench消息并存储力和力矩
            wrench_msg = Wrench()

            # 设置力（force）字段
            wrench_msg.force = Vector3()
            wrench_msg.force.x = fx
            wrench_msg.force.y = fy
            wrench_msg.force.z = fz

            # 设置力矩（torque）字段
            wrench_msg.torque = Vector3()
            wrench_msg.torque.x = mx
            wrench_msg.torque.y = my
            wrench_msg.torque.z = mz

            # 发布消息
            pub.publish(wrench_msg)

        # 控制发布频率
        rate.sleep()

    # 关闭串口
    ser.close()
