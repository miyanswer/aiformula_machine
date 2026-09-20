#!/usr/bin/env python3
"""
odom_imu_localizer.py - 車輪速 (CAN) + IMU ヨーレートの積算による自己位置 (デッドレコニング).

    速度 v    : CAN の車輪回転数フレーム (id=1809, data[0:4]=右 RPM, data[4:8]=左 RPM, int32 LE)
                v = (左 + 右)/2 * RPM/60 * pi * 車輪径   (odometry_publisher/wheel.hpp と同じ)
    ヨーレート: IMU (vectornav) の angular_velocity.z - ジャイロバイアス
    積算      : IMU 受信ごとに中点積分 (x += v cos(yaw + w dt/2) dt, ...)

ジャイロバイアスは「停止中 (車輪速 ~0 が stationary_time 秒続いている間)」に angular_velocity.z の
平均として推定し差し引く. 周回マップは方位ドリフトに弱いので, 走り出す前に数秒停止させると良い.

出力: nav_msgs/Odometry (frame: odom -> base_footprint). lane_navigator が購読する.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

RPM_ID = 1809


class OdomImuLocalizer(Node):
    def __init__(self):
        super().__init__("odom_imu_localizer")
        d = self.declare_parameter
        self.can_topic = d("can_topic", "/aiformula_sensing/vehicle_info").value
        self.imu_topic = d("imu_topic", "/aiformula_sensing/vectornav/imu").value
        self.odom_topic = d("odom_topic", "/aiformula_sensing/odom_imu_localizer/odom").value
        self.odom_frame_id = d("odom_frame_id", "odom").value
        self.child_frame_id = d("child_frame_id", "base_footprint").value
        self.wheel_diameter = float(d("wheel.diameter", 0.254).value)
        self.publish_tf = bool(d("publish_tf", False).value)
        self.publish_rate = float(d("publish_rate", 50.0).value)
        self.speed_timeout = float(d("speed_timeout", 0.3).value)
        self.stationary_speed = float(d("stationary_speed", 0.02).value)
        self.stationary_time = float(d("stationary_time", 1.0).value)
        self.bias_alpha = float(d("gyro_bias_alpha", 0.01).value)
        self.estimate_gyro_bias = bool(d("estimate_gyro_bias", True).value)

        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.bias = 0.0
        self.last_speed_time = None
        self.stationary_since = None
        self.last_imu_time = None

        self.create_subscription(Frame, self.can_topic, self._can_cb, 50)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 100)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            f"odom_imu_localizer: CAN={self.can_topic} IMU={self.imu_topic} -> {self.odom_topic} "
            f"(wheel diameter {self.wheel_diameter} m, gyro bias estimation={self.estimate_gyro_bias})")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _can_cb(self, msg: Frame):
        if msg.id != RPM_ID or len(msg.data) < 8:
            return
        data = bytes(msg.data)
        rpm_right, rpm_left = struct.unpack("<ii", data[:8])
        circumference = math.pi * self.wheel_diameter
        self.v = 0.5 * (rpm_left + rpm_right) / 60.0 * circumference
        self.last_speed_time = self._now()

    def _imu_cb(self, msg: Imu):
        now = self._now()
        wz = msg.angular_velocity.z
        # 車輪速が途絶えたら 0 扱い (古い速度で積算し続けない)
        v = self.v if self.last_speed_time is not None and now - self.last_speed_time < self.speed_timeout else 0.0

        stationary = abs(v) < self.stationary_speed
        if stationary:
            self.stationary_since = self.stationary_since or now
            if self.estimate_gyro_bias and now - self.stationary_since >= self.stationary_time:
                self.bias += self.bias_alpha * (wz - self.bias)
        else:
            self.stationary_since = None

        omega = 0.0 if stationary else wz - self.bias
        if self.last_imu_time is not None:
            dt = min(max(now - self.last_imu_time, 0.0), 0.1)
            yaw_mid = self.yaw + 0.5 * omega * dt
            self.x += v * math.cos(yaw_mid) * dt
            self.y += v * math.sin(yaw_mid) * dt
            self.yaw = math.atan2(math.sin(self.yaw + omega * dt), math.cos(self.yaw + omega * dt))
        self.last_imu_time = now
        self.omega = omega
        self._v_used = v

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = getattr(self, "_v_used", 0.0)
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
