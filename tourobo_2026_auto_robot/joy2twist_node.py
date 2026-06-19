"""joyを受け取って、twistをpubilshする
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy, Imu
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

import tf_transformations
from tf_transformations import euler_from_quaternion

from dyna_interfaces.msg import DynaFeedback, DynaTarget


def rot(vec, theta):
    vec = vec.reshape(-1, 1)

    cos, sin = np.cos(theta), np.sin(theta)

    R = np.array([
        [cos, -sin],
        [sin, cos],
    ])

    return R @ vec


class Joy2Twist(Node):

    def __init__(self):
        super().__init__("joy2twist")
        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )
        self.subscription_joy

        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.subscription_imu = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_callback,
            10,
        )
        self.subscription_imu
        self.yaw_rad = 0

        # -----robot parameter-----
        self.twist_x_gain = 1.0
        self.twist_y_gain = 1.0
        self.omega_gain = 1.0

    def imu_callback(self, msg):
        q = msg.orientation

        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_rad = euler[2]

    def joy_callback(self, msg):
        axes_values = msg.axes

        if abs(axes_values[1]) < 0.1:
            axes_values[1] = 0
        if abs(axes_values[0]) < 0.1:
            axes_values[0] = 0
        if abs(axes_values[3]) < 0.1:
            axes_values[3] = 0

        twist_y = axes_values[0] * self.twist_y_gain
        twist_x = axes_values[1] * self.twist_x_gain
        w = axes_values[3] * self.omega_gain

        twist = Twist()
        v = np.array([twist_x, twist_y])
        Rv = rot(v, self.yaw_rad)

        #twist.linear.x = float(Rv[1])
        #twist.linear.y = float(Rv[0])
        #twist.angular.z = axes_values[3]

        twist.linear.x = twist_x
        twist.linear.y = twist_y
        twist.angular.z = w

        self.twist_publisher.publish(twist)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    joy2twist_node = Joy2Twist()

    rclpy.spin(joy2twist_node)  # ノードをスピンさせる
    joy2twist_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
