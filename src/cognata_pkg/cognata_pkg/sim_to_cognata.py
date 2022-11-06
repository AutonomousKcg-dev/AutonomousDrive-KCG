import rclpy
from rclpy.node import Node
from cognata_sdk_ros2.msg import GPSAdditionalData
from autoware_auto_msgs.msg import VehicleControlCommand, VehicleKinematicState
from autoware_auto_msgs.msg import Complex32
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32



class ConvToCognata(Node):

    def __init__(self):

        super().__init__('convToCognata')

        # subscribers to Simulator
        self.create_subscription(VehicleControlCommand, "/raw_command", self.accel_cb, 10)
        
        # publishers to Cognata
        self.accel_pub = self.create_publisher(Float32, "/cognataSDK/car_command/acceleration_cmd", 10)
        self.steer_pub = self.create_publisher(Float32, "/cognataSDK/car_command/steer_cmd", 10)
        
        # Vehicle variables
        self.acc = 0.0
        self.steer_ang = 0.0


    
    def accel_cb(self, msg: VehicleControlCommand):
        """
            Callback function that receive the Vehicle raw command from the simulator and 
            publish it to cognata's simulator
        """
        # extract values

        self.acc = Float32()
        self.steer_ang = Float32()
        self.acc.data = msg.long_accel_mps2
        self.steer_ang.data = msg.front_wheel_angle_rad

        # publish the commands
        self.accel_pub.publish(self.acc)
        self.steer_pub.publish(self.steer_ang)


def main(args=None):
    rclpy.init(args=args)

    conv = ConvToCognata()

    rclpy.spin(conv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        




        




