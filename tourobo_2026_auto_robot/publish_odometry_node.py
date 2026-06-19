"""IMU,tofセンサ2つ、接地エンコーダー2つをesp32からシリアル通信で受け取り、publishする"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from tf2_ros import TransformBroadcaster

# メッセージ型
from std_msgs.msg import String, UInt16
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from recv_feedback import *


def calc_delta_odometry(x_vel, y_vel, theta, ang_z_vel, dt):
    delta_x = (x_vel * math.cos(theta) - y_vel * math.sin(theta)) * dt
    delta_y = (x_vel * math.sin(theta) + y_vel * math.cos(theta)) * dt
    delta_theta = ang_z_vel * dt

    return (delta_x, delta_y, delta_theta)


class feedback_publisher(Node):

    def __init__(self):
        super().__init__("feedback_publisher")

        # --- パラメータ設定 ----
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("wheel_radius", 0.0235)  #[m]
        self.declare_parameter("port_name", "/dev/ttyACM0")

        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        port_name = self.get_parameter("port_name").value

        # --- publisherの設定 ---
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tof1_pub = self.create_publisher(UInt16, "/tof_1", 10)
        self.tof2_pub = self.create_publisher(UInt16, "/tof_2", 10)
        self.tof3_pub = self.create_publisher(UInt16, "/tof_3", 10)
        self.tof4_pub = self.create_publisher(UInt16, "/tof_4", 10)

        # recv_feedbackの割り込み設定
        self.recv_feedback_timer = self.create_timer(0.007, self.recv_feedback)

        # esp32 serialの設定
        self.ser = serial.Serial(port=port_name, baudrate=10000000, timeout=0.0)
        self.ser.low_latency = True
        # メンバ変数初期化
        self.mcu_timestamp_millis = 0

        self.enc_x_vel = 0.0
        self.enc_y_vel = 0.0

        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        self.ang_x_vel = 0.0
        self.ang_y_vel = 0.0
        self.ang_z_vel = 0.0

        self.tof1 = 0
        self.tof2 = 0
        self.tof3 = 0
        self.tof4 = 0

        self.last_time = self.get_clock().now()

        # ---- Params -----
        self.Lx = 0.313  #中心からy軸odomまでの距離[m]
        self.y_enc_alpha = 16.7  #y方向エンコーダーの中心からの角度 [degree]

    def publish_feedback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # tofの配信
        tof1_msg = UInt16()
        tof1_msg.data = int(self.tof1)
        self.tof1_pub.publish(tof1_msg)

        tof2_msg = UInt16()
        tof2_msg.data = int(self.tof2)
        self.tof2_pub.publish(tof2_msg)

        tof3_msg = UInt16()
        tof3_msg.data = int(self.tof3)
        self.tof3_pub.publish(tof3_msg)

        tof4_msg = UInt16()
        tof4_msg.data = int(self.tof4)
        self.tof4_pub.publish(tof4_msg)

        # odomの配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        # 位置情報 (積分計算はekfが行う)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        # 共分散(位置)
        odom.pose.covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,  # x
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,  # y
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,  # z
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,  # roll
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,  # pitch
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-3,  # yaw
        ]

        # 速度情報 (角速度は0)
        wheel_x_vel = self.enc_x_vel * 2 * math.pi * self.wheel_radius  # [m/s]
        wheel_y_vel = -(self.enc_y_vel * 2 * math.pi * self.wheel_radius)
        alpha = np.deg2rad(self.y_enc_alpha)

        #euler = tf_transformations.euler_from_quaternion(
        # ---- ODOM計算式(ekfを使う場合は不要)
        #    [self.q_x, self.q_y, self.q_z, self.q_w])
        #yaw = euler[2]
        ##self.get_logger().info(f"yaw{yaw}")
        #cos, sin = np.cos(yaw), np.sin(yaw)

        #R = np.array([[cos, -sin, self.Lx * np.cos(alpha) * sin],
        #              [sin, cos, -self.Lx * np.cos(alpha) * cos]])

        #vec = np.array([wheel_x_vel, wheel_y_vel, self.ang_z_vel])
        #twist_vec = R @ vec

        #odom.twist.twist.linear.x = twist_vec[0]
        #odom.twist.twist.linear.y = twist_vec[1]
        #odom.twist.twist.angular.z = 0.0

        odom.header.stamp = current_time.to_msg()
        odom.twist.twist.linear.x = wheel_x_vel
        odom.twist.twist.linear.y = wheel_y_vel - self.Lx * np.cos(
            alpha) * self.ang_z_vel
        odom.twist.twist.angular.z = 0.0

        # 共分散(Twist)
        odom.twist.covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,  # x速度の信頼度
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,  # y速度の信頼度
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-3,  # yaw角速度の信頼度
        ]

        self.odom_pub.publish(odom)

        # imuの配信
        imu = Imu()
        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = self.imu_frame_id

        # 姿勢(Quaternion)
        imu.orientation.w = self.q_w
        imu.orientation.x = self.q_x
        imu.orientation.y = self.q_y
        imu.orientation.z = self.q_z

        # 共分散行列
        # 本来はセンサーのスペックから設定
        imu.orientation_covariance = [
            1e-9, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 1e-9
        ]

        # 角速度(Aungular Vel)
        imu.angular_velocity.x = self.ang_x_vel
        imu.angular_velocity.y = self.ang_y_vel
        imu.angular_velocity.z = self.ang_z_vel
        imu.angular_velocity_covariance = [
            1e-9,
            0.0,
            0.0,
            0.0,
            1e-9,
            0.0,
            0.0,
            0.0,
            1e-9,
        ]

        self.imu_pub.publish(imu)

        # tfはsensor fusionパッケージで出力する

    def recv_feedback(self):
        struct_format = "<BIfffffffffHHHHB"
        packet = receive_packet(struct_format, self.ser)
        if packet is None:
            return

        self.mcu_timestamp_millis = packet[1]

        self.q_w = packet[2]
        self.q_x = packet[3]
        self.q_y = packet[4]
        self.q_z = packet[5]

        self.ang_x_vel = packet[6]
        self.ang_y_vel = packet[7]
        self.ang_z_vel = packet[8]

        self.enc_x_vel = packet[9]  #[rot/s]
        self.enc_y_vel = packet[10]

        self.tof1 = packet[11]  #前方
        self.tof2 = packet[12]
        self.tof3 = packet[13]

        self.tof4 = packet[14]  #後方

        self.publish_feedback()


def main():
    rclpy.init()  # rclpyライブラリの初期化

    feedback_publisher_node = feedback_publisher()
    rclpy.spin(feedback_publisher_node)  # ノードをスピンさせる
    feedback_publisher_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
