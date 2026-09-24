#!/usr/bin/env python
import time
from typing import List
import numpy as np
from enum import IntEnum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame

from common_python.get_ros_parameter import get_ros_parameter


class DriveWheel(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_DRIVE_WHEELS = 2


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.get_ros_params()

        # Publisher & Subscriber
        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, buffer_size)
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.publish_timer = self.create_timer(self.publish_timer_loop_duration, self.publish_canframe_callback)

        self.can_sub = self.create_subscription(Frame, 'sub_can', self.can_receive_callback, buffer_size)

        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x210                      # MotorController CAN ID : 0x210
        self.frame_msg.dlc = 8                         # Data length

        # 指令タイムアウト (安全策): twist_mux は入力が全て途絶えても何も publish
        # しないため、この watchdog が無いと最後に受けた速度指令を CAN に送り
        # 続ける (Mac 遠隔操作中に Wi-Fi が切れると、その速度のまま走り続ける)。
        # cmd_timeout 秒 cmd が来なければ目標速度を 0 にする。未受信の起動直後も 0。
        self.last_cmd_time = None
        self.cmd_timed_out = True

        # 加減速制限: 受けた (v, omega) は目標値として保持し、タイマー周期ごとに
        # max_linear_accel / max_linear_decel / max_angular_accel を超えない範囲で
        # 近づけた値を RPM に変換して送る。停止指令やタイムアウト時もいきなり
        # RPM 0 にせず一定減速度で止める (急停止によるスリップ・転倒の防止)。
        # web_simulator/js/vehicle_physics.js の MOTOR_* 定数と同じ値にすること。
        self.target_v = 0.0
        self.target_omega = 0.0
        self.current_v = 0.0
        self.current_omega = 0.0
        self.last_tick_time = None
        self.frame_msg.data = self.toCanData(0.0, 0.0)

    def get_ros_params(self):
        self.diameter = get_ros_parameter(self, "wheel.diameter")
        self.tread = get_ros_parameter(self, "wheel.tread")
        self.gear_ratio = get_ros_parameter(self, "wheel.gear_ratio")
        self.publish_timer_loop_duration = get_ros_parameter(self, "publish_timer_loop_duration")
        self.cmd_timeout = get_ros_parameter(self, "cmd_timeout")
        self.max_linear_accel = get_ros_parameter(self, "max_linear_accel")
        self.max_linear_decel = get_ros_parameter(self, "max_linear_decel")
        self.max_angular_accel = get_ros_parameter(self, "max_angular_accel")

    def twist_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_omega = msg.angular.z
        self.last_cmd_time = time.monotonic()
        if self.cmd_timed_out:
            self.get_logger().info("Speed command received.")
        self.cmd_timed_out = False

    def publish_canframe_callback(self):
        now = time.monotonic()
        if not self.cmd_timed_out and now - self.last_cmd_time > self.cmd_timeout:
            self.cmd_timed_out = True
            self.target_v = 0.0
            self.target_omega = 0.0
            self.get_logger().warn(
                f"No speed command for {self.cmd_timeout:.2f} s -> stopping (command timeout)")

        # 実際の経過時間で積分 (タイマー遅延時に制限を超えないよう上限あり)
        dt = self.publish_timer_loop_duration if self.last_tick_time is None \
            else min(now - self.last_tick_time, 0.1)
        self.last_tick_time = now
        self.current_v = self.rateLimitLinear(self.current_v, self.target_v, dt)
        self.current_omega = self.rateLimit(
            self.current_omega, self.target_omega, self.max_angular_accel * dt)
        self.frame_msg.data = self.toCanData(self.current_v, self.current_omega)
        self.can_pub.publish(self.frame_msg)

    @staticmethod
    def rateLimit(current: float, target: float, max_step: float) -> float:
        return current + max(-max_step, min(max_step, target - current))

    def rateLimitLinear(self, current: float, target: float, dt: float) -> float:
        # |v| が小さくなる向き (減速・停止・前後反転の手前まで) は max_linear_decel、
        # 大きくなる向きは max_linear_accel で制限する。
        if current * target < 0.0:
            return self.rateLimit(current, 0.0, self.max_linear_decel * dt)
        if abs(target) < abs(current):
            return self.rateLimit(current, target, self.max_linear_decel * dt)
        return self.rateLimit(current, target, self.max_linear_accel * dt)

    def toCanData(self, linear_velocity: float, angular_velocity: float) -> List[int]:
        rpm = self.toRefRPM(linear_velocity, angular_velocity)
        return self.toCanCmd(rpm[DriveWheel.RIGHT]) + self.toCanCmd(rpm[DriveWheel.LEFT])

    # Feedback CAN Frame reception
    def can_receive_callback(self, msg: Frame):
        if msg.id == 0x211:  # Assume this is the feedback CAN ID
            rpm_left = int.from_bytes(msg.data[0:4], byteorder='little', signed=True)
            rpm_right = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
            self.get_logger().info(f"Feedback: LEFT RPM = {rpm_left}, RIGHT RPM = {rpm_right}")

    # Velocity -> RPM Calc
    def toRefRPM(self, linear_velocity, angular_velocity):
        wheel_angular_velocities = np.zeros(DriveWheel.NUM_DRIVE_WHEELS)

        wheel_angular_velocities[DriveWheel.LEFT] = (
            linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity  # [rad/s]

        wheel_angular_velocities[DriveWheel.RIGHT] = (
            linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity  # [rad/s]

        minute_to_second = 60.
        rpm = wheel_angular_velocities * (minute_to_second / (2. * np.pi))
        if rpm[DriveWheel.LEFT] * rpm[DriveWheel.RIGHT] < 0.0:
            rpm[:] = 0.0
            self.get_logger().debug(f"Preventing in-situ rotation ! (rpm: {rpm})")
        return (rpm * self.gear_ratio).tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()