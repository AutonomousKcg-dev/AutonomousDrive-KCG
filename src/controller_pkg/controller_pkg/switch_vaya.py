# Ros Topics Data types - Vaya
from itertools import count
import time
from typing import List
import rclpy
from rclpy.node import Node
from tracked_object_msgs.msg import TrackedObjectArray, TrackedObject
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Pose, Point
from std_msgs.msg import Float32, Int8, String

# Idan driver
from autoware_auto_msgs.msg import VehicleControlCommand, VehicleKinematicState

# Generic Imports
import enum
import math
import numpy as np
import cv2

"""
    Auxuliary Classes
"""


class front_padastrian:
    id = -1
    distance_x = 100
    distance_y = 0
    velocity_x = 0
    velocity_y = 0


class front_car:
    id = -1
    distance_x = 100
    distance_y = 0
    velocity_x = 0
    velocity_y = 0


class State(enum.Enum):
    ACC = 1
    BRAKE = 2
    LEFT = 3
    RIGHT = 4
    EMERGANCY = 5


class Direction(enum.Enum):
    RIGHT = 1
    LEFT = -1


"""
    Main Class
"""


def _get_front_object(obj_list: List[TrackedObject]) -> TrackedObject:
    """
    given a list of objects find the closest object
    """
    if not obj_list:
        return None

    index = np.argmin([math.dist((0, 0), (obj.object_pose_m_quat.position.x,
                      obj.object_pose_m_quat.position.y)) for obj in obj_list])

    return obj_list[index]


class switch_vaya(Node):

    def __init__(self, ref_dist=25):
        super().__init__("switch")
        # Subscribers
        self.create_subscription(
            TrackedObjectArray, "/tracked_objects", self.objects_message_callback, 10)
        self.create_subscription(
            OccupancyGrid, "/occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(VehicleKinematicState, "vehicle_state", self.velo_state_cb, 10)

        # Publishers
        self.switch_cmd_pub = self.create_publisher(Int8, "switch_cmd", 10)
        self.timer = self.create_timer(0.1, self.run)
        self.pub_traj = self.create_publisher(String, "lane_switch", 10)
        self.test_pub = self.create_publisher(Int8, "test_topic", 10)

        # Object section
        self.pedestrian_list = []
        self.vehicle_list = []
        self.front_car: front_car = None
        self.front_ped: front_padastrian = None

        # Lanes
        self.switch_lane_direction = Direction.LEFT     # default

        # ego car information
        self.speed = 0.0
        self.radius = 8.0
        self.state = State.ACC  # ACC is the starting state
        # the distance the car should keep from its front car
        self.refernce_distance = ref_dist
        self.side_lane_look_ahead = (-30, 60)  # meters
        self.min_speed = 5     # front car should be atleast that speed km/h
        self.changed_lane = True

        #couter for speed controler
        self.couter = 0

        # start time
        self._start_time = time.time()
        self.switch = False

        # Occupancy Grid Params
        self.colormap = [[0,0,0],[0,240,0],[0,100,0],[120,0,0],[255,0,0]]+[[255,255,255] for i in range(251)]
        self.color_lookup = np.array(self.colormap, dtype=np.uint8)
        self.channels = 3
    
    def change_color(self, number):
        return self.colormap[number]

    def velo_state_cb(self,msg:VehicleKinematicState):
        #v_state
        self.v_state = msg
        self.speed = self.v_state.state.longitudinal_velocity_mps

    def objects_message_callback(self, msg: TrackedObjectArray):
        """
            Same method as DOGTcb from cognata.
            update the vehicle list and pedestrian list.
            also update the switch.
        """
        self.vehicle_list.clear()
        self.pedestrian_list.clear()

        vehicle_list = []
        pedestrian_list = []
        for Tobject in msg.tracked_objects:
            Tobject: TrackedObject = Tobject
            if Tobject.object_type == 11000:
                # Pedestrian
                pedestrian_list.append(Tobject)

            elif Tobject.object_type == 20000:
                # Vehicle
                vehicle_list.append(Tobject)

        self.vehicle_list = vehicle_list
        self.pedestrian_list = pedestrian_list


    def occupancy_grid_callback(self, msg: OccupancyGrid):
        """
            This method is responsible for GUI & Occupancy Grid data update
        """
        if len(list(msg.data)) > 0:
            data = map(self.change_color, list(msg.data))
            grid = np.array(list(data), dtype=np.uint8)
            print("all zeros: ", all(np.equal(grid.flatten(), np.zeros_like(grid).flatten())))
            grid = np.reshape(grid, (msg.info.height, msg.info.width, 3))
            grid = cv2.resize(grid, (600, 600))
            cv2.imshow("frame", grid)
            cv2.waitKey(1)

    def object_priority(self, front_car: front_car, front_ped: front_padastrian) -> bool:
        # find the closest object
        return (math.dist((0, 0), (front_car.distance_x, front_car.distance_y)) >
                math.dist((0, 0), (front_ped.distance_x, front_ped.distance_y)))

    def in_area(self, area_x, area_y, object: TrackedObject, direction: str) -> bool:
        """
            this method checks wheter the object is in the given area
        """
        center_point = (object.object_pose_m_quat.position.x,
                        object.object_pose_m_quat.position.y)
        if direction == 'left':
            if area_y[0] <= center_point[0] <= area_y[1]:
                if area_x[0] <= center_point[1] <= area_x[1]:
                    return True
            return False
        else:
            if area_y[0] <= center_point[0] <= area_y[1]:
                if area_x[0] >= center_point[1] >= area_x[1]:
                    return True
            return False

    def switch_lane_is_possible(self) -> bool:
        """
            This method checks if it is possible to switch lane, 
            return True if it is and update the direction of the turn.
            else, return False.
        """
        # decide if switch lane is possible, decide which direction to take (left, right)
        left_area_x = (self.radius/2, self.radius*(3/2))
        left_area_y = (
            self.side_lane_look_ahead[0], self.side_lane_look_ahead[1])
        left_free = True

        right_area_x = (-self.radius/2, -self.radius * (3/2))
        right_area_y = (
            self.side_lane_look_ahead[0], self.side_lane_look_ahead[1])

        # first, check left area
        for car in self.vehicle_list:
            if self.in_area(left_area_x, left_area_y, car, 'left'):
                left_free = False

        for ped in self.pedestrian_list:
            if self.in_area(left_area_x, left_area_y, ped, 'left'):
                # both lanes aren't free
                left_free = False

        if left_free:
            # choose left lane
            self.switch_lane_direction = Direction.LEFT
            print("LEFT TURN")
            return True

        # Second, check for the right lane
        for car in self.vehicle_list:
            if self.in_area(right_area_x, right_area_y, car, 'right'):
                # both lanes aren't free
                print("CANT SWITCH LANE")
                return False

        for ped in self.pedestrian_list:
            if self.in_area(right_area_x, right_area_y, ped, 'right'):
                # both lanes aren't free
                print("CANT SWITCH LANE")
                return False

        # the right lane is free
        self.switch_lane_direction = Direction.RIGHT
        print("RIGHT TURN")
        return True

    def time_2_impact(self, front_object) -> float:
        
        time_to_impact = 20

        if front_object:
            # get front object velocity
            front_object_speed = np.linalg.norm([
                front_object.object_velocity_mps_radps.linear.x,
                front_object.object_velocity_mps_radps.linear.y,
                front_object.object_velocity_mps_radps.linear.z
            ])  # m/s (a guess)

            front_object_distance = np.linalg.norm([
                front_object.object_pose_m_quat.position.x,
                front_object.object_pose_m_quat.position.y,
                front_object.object_pose_m_quat.position.z
            ])

            # calculate time to impact
            time_to_impact = (front_object_distance /
                              (max(self.speed, 1.5) - front_object_speed))
        
        return time_to_impact


    # TODO not in [m] -> in time to accident
    # make decision
    def get_decision(self):

        areas = self.get_vehicle_areas_info()
        
        # get the front object
        front_object = _get_front_object(areas["mid"])
        final_decision = State.ACC
        time_to_imapct = 20.0

        if front_object:

            time_to_imapct = self.time_2_impact(front_object)

            # TODO Emmergancy if
            if time_to_imapct <= 2.5:
                final_decision = State.EMERGANCY

            # TODO Brake if
            elif 2.5 < time_to_imapct <= 5.5:
                final_decision = State.BRAKE
                
            # TODO Switch lane if
            elif 5.5 < time_to_imapct <= 10.0:
                lane = Int8()

                # Try right lane first
                if len(areas['right']) == 0:
                    final_decision = State.RIGHT
                    lane.data = 1
                elif len(areas['right']) > 0:
                    right_obj = _get_front_object(areas['right'])
                    time_to_imapct = self.time_2_impact(right_obj)
                    if time_to_imapct > 10.0:
                        final_decision = State.RIGHT
                        lane.data = 1

                    # Try left lane if right is taken
                    elif len(areas['left']) == 0:
                        final_decision = State.LEFT
                        lane.data = 2
                    elif len(areas['left']) > 0:
                        left_obj = _get_front_object(areas['left'])
                        time_to_imapct = self.time_2_impact(left_obj)
                        if time_to_imapct > 10.0:
                            final_decision = State.LEFT
                            lane.data = 2
                        else:
                            final_decision = State.ACC
                    
                self.test_pub.publish(lane)

            # TODO ACC
            else:
                final_decision = State.ACC

        self.get_logger().info("time to impact {} sec ,Our Speed: {} m/s, state {}".format(time_to_imapct ,self.speed, final_decision))        
        return final_decision

    # TODO return also the Vehicle themself so we can messuare time to conflict for braking ...

    def get_vehicle_areas_info(self):
        """
            Print Debug method.
        """
        areas = {'left': [], 'mid': [], 'right': []}
        num_of_objects = {'left': 0, 'mid': 0, 'right': 0}
        # decide if switch lane is possible, decide which direction to take (left, right)
        left_area_x = (self.radius/2, self.radius*(3/2))        # Actually Y

        # Actually X
        left_area_y = (
            self.side_lane_look_ahead[0], self.side_lane_look_ahead[1])

        right_area_x = (-self.radius/2, -self.radius *
                        (3/2))     # Actually Y

        # Actually X
        right_area_y = (
            self.side_lane_look_ahead[0], self.side_lane_look_ahead[1])

        mid_area_x = (-self.radius/2, self.radius/2)
        mid_area_y = (
            self.side_lane_look_ahead[0], self.side_lane_look_ahead[1])

        for car in self.vehicle_list:
            if self.in_area(left_area_x, left_area_y, car, 'left'):
                areas['left'].append(car)
                num_of_objects['left'] += 1
            elif self.in_area(right_area_x, right_area_y, car, 'right'):
                areas['right'].append(car)
                num_of_objects['right'] += 1
            elif self.in_area(mid_area_x, mid_area_y, car, 'left'):
                areas['mid'].append(car)
                num_of_objects['mid'] += 1

        for ped in self.pedestrian_list:
            if self.in_area(left_area_x, left_area_y, ped, 'left'):
                areas['left'].append(ped)
                num_of_objects['left'] += 1
            elif self.in_area(right_area_x, right_area_y, ped, 'right'):
                areas['right'].append(ped)
                num_of_objects['right'] += 1
            elif self.in_area(mid_area_x, mid_area_y, ped, 'left'):
                areas['mid'].append(ped)
                num_of_objects['mid'] += 1

        self.get_logger().info("{}".format(num_of_objects))
        # print(num_of_objects)
        return areas

    def run(self, front_car: front_car = None, front_ped: front_padastrian = None):
        """
            this method is the main method.
            Update the switch state for the decision making.
            TODO Sync with the actual switch lane.
        """
        
        self.front_car = front_car
        self.front_ped = front_ped

        state = self.get_decision()

        output = Int8()
        output.data = state.value
        self.switch_cmd_pub.publish(output)



def main(args=None):
    rclpy.init(args=args)

    switch = switch_vaya()

    rclpy.spin(switch)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    switch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
