import enum
from importlib.util import set_loader
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

# Idan driver
from autoware_auto_msgs.msg import VehicleControlCommand

class State(enum.Enum):
    ACC = 1
    BRAKE = 2
    LEFT = 3
    RIGHT = 4
    EMERGANCY = 5


class Controller(Node):
    """
        ACC = 0
        BRAKE = 1
        LEFT = 2
        RIGHT = 3
        EMERGANCY = 4
    """

    def __init__(self):
        super().__init__("controller")

        # subscribers
        self.create_subscription(Int8, "switch_cmd", self.switch_cb, 10)
        self.create_subscription(
            VehicleControlCommand, "raw_command_master", self.vel_pid_cb, 10)

        # publishers
        self.idan_pub = self.create_publisher(
            VehicleControlCommand, "raw_command", 10)
        self.timer = self.create_timer(0.1, self.control)

        # ego variables
        self.state = 0  # default ACC
        self.idan_msg: VehicleControlCommand = VehicleControlCommand()

    def switch_cb(self, msg: Int8):
        # update the current state of the switch
        self.state = msg.data

    def vel_pid_cb(self, msg: VehicleControlCommand):
        # update the idan command
        self.idan_msg = msg

    def control(self):
        """
            This method decieds the next step of the vehicle based on the given state
            from the switch.
        """
        # TODO Emergancy
        if self.state == State.EMERGANCY.value:
            self.idan_msg.long_accel_mps2 = -2.0
            self.get_logger().info("EBrake")
        # TODO Brake
        elif self.state == State.BRAKE.value:
            self.idan_msg.long_accel_mps2 = -1.0
            self.get_logger().info("Brake")

        # TODO Right
        elif self.state == State.RIGHT.value:
            # TODO change file (from here)
            # TODO and add the raw command from ACC
            self.get_logger().info("Moving to right lane")
        # TODO Left
        elif self.state == State.LEFT.value:
            # TODO change file (from here)
            # TODO and add the raw command from ACC
            self.get_logger().info("Moving to left lane")


        self.idan_pub.publish(self.idan_msg)


def main(args=None):
    rclpy.init(args=args)

    control = Controller()

    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
