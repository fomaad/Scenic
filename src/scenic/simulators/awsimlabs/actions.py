import math
import time
from enum import Enum

from scenic.domains.driving.actions import *
from scenic.core.geometry import normalizeAngle
import utils
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from autoware_vehicle_msgs.msg import Engage
from tier4_planning_msgs.msg import VelocityLimit
import std_msgs.msg

class AdsInternalStatus(Enum):
    UNINITIALIZED = 0
    LOCALIZATION_SUCCEEDED = 1
    GOAL_SET = 2
    AUTONOMOUS_MODE_READY = 3
    AUTONOMOUS_IN_PROGRESS = 4
    GOAL_ARRIVED = 5

    def __lt__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value < other.value
        return NotImplemented
    def __le__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value <= other.value
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value >= other.value
        return NotImplemented
    def __gt__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value > other.value
        return NotImplemented

class MotionState(Enum):
    """
    To make it consistent with Autoware, don't change the value
    uint16 UNKNOWN = 0
    uint16 STOPPED = 1
    uint16 STARTING = 2
    uint16 MOVING = 3
    """
    STOPPED = 1
    MOVING = 3

class SetDestinationAction(Action):
    def __init__(self, dest):
        self.dest = dest

    def canBeTakenBy(self, agent):
        return agent.isEgo

    def applyTo(self, obj, simulation):
        upd_pos = simulation.simulator.network.correct_elevation(self.dest)

        msg = PoseStamped()
        msg.header.stamp = simulation.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = utils.scenic_point_to_ros_point(upd_pos)

        quaternion = utils.yaw_to_quaternion(normalizeAngle(self.dest.heading + math.pi/2))
        msg.pose.orientation = quaternion

        simulation.ego_goal_publisher.publish(msg)
        rclpy.spin_once(simulation.node, timeout_sec=0.1)
        simulation.ads_internal_status = AdsInternalStatus.GOAL_SET
        print('set Ego destination done')

class SendEngageCommandAction(Action):
    def __init__(self):
        pass
    def canBeTakenBy(self, agent):
        return agent.isEgo

    def applyTo(self, obj, simulation):
        print("Sending engage command to activate the autonomous operation mode.")
        msg = Engage()
        msg.stamp = simulation.node.get_clock().now().to_msg()
        msg.engage = True

        simulation.ego_auto_engage_publisher.publish(msg)
        rclpy.spin_once(simulation.node, timeout_sec=0.1)
        simulation.ads_internal_status = AdsInternalStatus.AUTONOMOUS_IN_PROGRESS

class SetMaxSpeedAction(Action):
    def __init__(self, max_speed=None):
        self.max_speed = max_speed

    def canBeTakenBy(self, agent):
        return agent.isEgo

    def applyTo(self, obj, simulation):
        is_max_speed_defined = self.max_speed is not None
        if is_max_speed_defined and not simulation._destroyed:
            vel_limit_msg = VelocityLimit()
            vel_limit_msg.max_velocity = float(self.max_speed)
            vel_limit_msg.use_constraints = False
            vel_limit_msg.sender = ""
            simulation.ego_max_speed_publisher.publish(vel_limit_msg)
            rclpy.spin_once(simulation.node)
            # print('set max speed')

class FollowLaneAction(Action):
    def __init__(self, target_speed=None, acceleration=None, deceleration=None):
        # if target_speed is None, it follow the speed limit of the current lane
        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def canBeTakenBy(self, agent):
        return agent.isVehicle

    def applyTo(self, npc_obj, simulation):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        my_dict = {
            "target": npc_obj.name,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": self.acceleration if is_acceleration_defined else 0,
            "deceleration": self.deceleration if is_deceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
            "is_deceleration_defined": is_deceleration_defined
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        simulation.follow_lane_cmd_publisher.publish(msg)
        rclpy.spin_once(simulation.node, timeout_sec=0.1)
        print(f"Sent follow lane command to {npc_obj.name}")

class FollowWaypointsAction(Action):
    def __init__(self, waypoints, target_speed=None, acceleration=None, deceleration=None):
        self.waypoints = waypoints
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def canBeTakenBy(self, agent):
        return agent.isVehicle

    def applyTo(self, npc_obj, simulation):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        ros_wps = []
        print("Original waypoints: ")
        for waypoint in self.waypoints:
            print(f"{waypoint.position}, heading: {waypoint.toHeading()}")
            corrected_position = simulation.simulator.network.correct_elevation(waypoint)
            ros_wps.append(utils.scenic_point_to_dict(corrected_position))

        print("Corrected waypoints: ")
        for waypoint in ros_wps:
            print(f"{waypoint}")

        my_dict = {
            "target": npc_obj.name,
            "waypoints": ros_wps,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": self.acceleration if is_acceleration_defined else 0,
            "deceleration": self.deceleration if is_deceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
            "is_deceleration_defined": is_deceleration_defined
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        print(msg.data)
        simulation.follow_waypoints_cmd_publisher.publish(msg)
        rclpy.spin_once(simulation.node, timeout_sec=0.1)
        print(f"Sent follow waypoints command to {npc_obj.name}")
