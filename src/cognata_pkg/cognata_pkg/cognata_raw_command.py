import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from autoware_auto_msgs.msg import VehicleControlCommand
from sensor_msgs.msg import Imu
from cognata_sdk_ros2.msg import GPSAdditionalData

import numpy as np


class CognataRawCommand(Node):
    def __init__(self):
        super().__init__('cognata_raw_command')

        # subscriber
        self.create_subscription(
            Imu, "/cognataSDK/GPS/imu/CognataGPS", self.acceleration_callback, 10)
        # self.create_subscription(
        #     Float32, "/cognataSDK/car_command/acceleration_cmd", self.acceleration_callback, 10)
        self.create_subscription(
            Float32, "/cognataSDK/car_command/steer_cmd", self.steer_callback, 10)

        # publisher
        self._vehicle_command_publisher = self.create_publisher(
            VehicleControlCommand, "raw_command", 10)

        # timed callback
        self.create_timer(0.1, self._pub)

        # message
        self._vehicle_command = VehicleControlCommand()

    def acceleration_callback(self, msg: Imu):
        self._vehicle_command.long_accel_mps2 = msg.linear_acceleration.x
        

    def steer_callback(self, msg: Float32):
        self._vehicle_command.front_wheel_angle_rad = msg.data
    
    def _pub(self):
        self._vehicle_command_publisher.publish(self._vehicle_command)

def main(args=None):
    rclpy.init(args=args)

    cognata_raw_command = CognataRawCommand()

    rclpy.spin(cognata_raw_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cognata_raw_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
