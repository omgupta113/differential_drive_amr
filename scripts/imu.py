#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
from adafruit_bno08x_rvc import BNO08x_RVC
import tf_transformations

ori_cov_ = 0.000001
accel_cov_ = 0.000001

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        timer_period = 1/30.0  # 30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize UART and BNO08x
        self.uart = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.1)
        self.rvc = BNO08x_RVC(self.uart)
        
    def timer_callback(self):
        try:
            yaw, pitch, roll, x_accel, y_accel, z_accel = self.rvc.heading
        except Exception as e:
            self.get_logger().error(f"IMU Read Error: {e}", throttle_duration_sec=5)
            return
        
        # Convert ENU yaw to ROS FLU frame
        yaw_adj = 90.0 - yaw
        yaw_rad = math.radians(yaw_adj)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        # Convert Euler to quaternion (ZYX extrinsic)
        quat = tf_transformations.quaternion_from_euler(
            yaw_rad, pitch_rad, roll_rad, axes='szyx'
        )
        
        # Prepare IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation
        imu_msg.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        imu_msg.orientation_covariance = [ori_cov_, 0.0, 0.0,
                                          0.0, ori_cov_, 0.0,
                                          0.0, 0.0, ori_cov_]
        
        # Linear acceleration (swap axes if needed)
        imu_msg.linear_acceleration = Vector3(
            x=y_accel,  # IMU Y is robot X
            y=x_accel,  # IMU X is robot Y
            z=z_accel
        )
        imu_msg.linear_acceleration_covariance = [accel_cov_, 0.0, 0.0,
                                                0.0, accel_cov_, 0.0,
                                                0.0, 0.0, accel_cov_]
        
        self.publisher_.publish(imu_msg)
        self.get_logger().debug('Published IMU data', throttle_duration_sec=1)
        
    def __del__(self):
        if hasattr(self, 'uart'):
            self.uart.close()

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()