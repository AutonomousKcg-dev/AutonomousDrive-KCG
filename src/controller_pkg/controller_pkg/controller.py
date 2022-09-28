import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

# Idan driver
from autoware_auto_msgs.msg import VehicleControlCommand


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
        self.create_subscription(VehicleControlCommand, "raw_command_master", self.vel_pid_cb, 10)

        # publishers
        self.idan_pub = self.create_publisher(VehicleControlCommand, "raw_command", 10)
        self.timer = self.create_timer(10, self.control)

        # ego variables

        self.state = 0  # default ACC
        self.idan_msg: VehicleControlCommand = None



    def switch_cb(self, msg: Int8):
        # update the current state of the switch
        self.state = msg.data
    
    def vel_pid_cb(self, msg: VehicleControlCommand):
        # update the idan command
        self.idan_msg = msg

    def control(self):
        """
            This method decied the next step of the vehicle base on the given state
            from the switch.
        """
        
        # Left - change lane to left
        if self.state == 2:
            # TODO
            pass
        
        # Right - change lane to left
        elif self.state == 3:
            # TODO
            pass
        
        # Emergancy - publish negative acceleration 
        elif self.state == 4:
            #   TODO check if -1.0 is possible for emergancy
            self.idan_msg.long_accel_mps2 = -0.4

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
