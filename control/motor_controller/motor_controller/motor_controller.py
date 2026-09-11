#!/usr/bin/env python
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

    def get_ros_params(self):
        self.diameter = get_ros_parameter(self, "wheel.diameter")
        self.tread = get_ros_parameter(self, "wheel.tread")
        self.gear_ratio = get_ros_parameter(self, "wheel.gear_ratio")
        self.publish_timer_loop_duration = get_ros_parameter(self, "publish_timer_loop_duration")

    def twist_callback(self, msg):
        rpm = self.toRefRPM(msg.linear.x, msg.angular.z)
        cmd_left = self.toCanCmd(rpm[DriveWheel.LEFT])
        cmd_right = self.toCanCmd(rpm[DriveWheel.RIGHT])
        can_data = cmd_right + cmd_left
        self.frame_msg.data = can_data

    def publish_canframe_callback(self):
        self.can_pub.publish(self.frame_msg)

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