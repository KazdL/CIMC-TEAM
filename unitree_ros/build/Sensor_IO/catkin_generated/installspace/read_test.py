import rospy
from geometry_msgs.msg import Wrench, Vector3
import struct
import time
import math

if __name__ == "__main__":
    rospy.init_node("publisher_test_node")
    pub = rospy.Publisher("wrench", Wrench, queue_size=10)
    rate = rospy.Rate(10)  # 每秒10次发布
    count = 0

    while not rospy.is_shutdown():
        count += 1
        count = count % 100

        fx = 0.0
        fy = 0.0
        fz = 1.0 * math.sin(math.pi/50*count)
        mx = 0.0
        my = 0.0
        mz = 0.0

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