"""twistをsubscribeして、足回りesp32にモーター指令値を送信する"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

#足回り用(id 2)エンコーダーピン配列
#const int ENC_PINNUM_A[4] = {19, 17, 21, 15};  足回りのピン
#const int ENC_PINNUM_B[4] = {23, 18, 16, 2};


def from_twist_to_motor_vel(vx, vy, w, L, fy):
    V_1 = (-vx - -vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_2 = (vx + vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_3 = (vx - vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_4 = (vx + vy + 2 * math.sqrt(2) * -w * L) / (4 * math.pi * fy)

    return (V_1, V_2, V_3, V_4)


bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


class TwistSubscriber(Node):

    def __init__(self):
        super().__init__("TwistSubscriber")

        # 足回り速度制御立ち上げ
        set_enc_vel_mode(0x010, bus)
        set_enc_vel_mode(0x011, bus)
        set_enc_vel_mode(0x012, bus)
        set_enc_vel_mode(0x013, bus)

        set_vel_pid_gain(0x010, 20, 4000, 0, bus)
        set_vel_pid_gain(0x011, 20, 4000, 0, bus)
        set_vel_pid_gain(0x012, 20, 4000, 0, bus)
        set_vel_pid_gain(0x013, 20, 4000, 0, bus)

        self.subscription_twist = self.create_subscription(
            Twist,  # メッセージの型
            "/cmd_vel",  # 購読するトピック名
            self.twist_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)

        self.subscription_twist

        # --- Config ---
        # 車体横の長さ
        self.L = 0.3
        # 車体中心からタイヤまでの距離
        self.fy = 0.127
        self.wheel_r = 0.05  #[m]
        self.twist_gain = 2 * math.pi * self.wheel_r

        # メンバーの初期化
        self.linear_x = 0
        self.linear_y = 0
        self.w = 0

        timer_period = 0.01
        # wirte_to_motorの割り込み設定
        self.timer = self.create_timer(timer_period, self.write_to_motor)

    def twist_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.w = msg.angular.z

    def write_to_motor(self):
        """Twistをメカナムホイール逆運動学で、各モーターの速度指令値に分解。4つの速度指令値を一つのパケットでesp32に送信する"""
        vx = self.linear_x
        vy = self.linear_y
        w = self.w

        V_1, V_2, V_3, V_4 = from_twist_to_motor_vel(vx, vy, w, self.L, self.fy)

        set_goal_vel(0x010, float(V_1 / self.twist_gain), bus)
        set_goal_vel(0x011, float(V_2 / self.twist_gain), bus)
        set_goal_vel(0x012, float(V_3 / self.twist_gain), bus)
        set_goal_vel(0x013, float(V_4 / self.twist_gain), bus)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = TwistSubscriber()

    rclpy.spin(twist_subscriber_node)
    twist_subscriber_node.destroy_node()
    rclpy.shutdown()


def stop():
    """停止モードにする"""
    set_stop_mode(0x010, bus)
    set_stop_mode(0x011, bus)
    set_stop_mode(0x012, bus)
    set_stop_mode(0x013, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()
