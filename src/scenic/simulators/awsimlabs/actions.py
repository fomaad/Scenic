import math
import time
from enum import Enum

from scenic.core.distributions import *
from scenic.domains.driving.actions import *
from scenic.core.geometry import normalizeAngle
import utils
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from autoware_vehicle_msgs.msg import Engage
from tier4_planning_msgs.msg import VelocityLimit
import std_msgs.msg
from aw_monitor.srv import *


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

class SetDestinationAction(Action):
    def __init__(self, dest):
        self.dest = dest

    def canBeTakenBy(self, agent):
        return agent.isEgo

    def applyTo(self, obj, simulation):
        upd_pos = simulation.simulator.network.correct_elevation(self.dest)

        msg = PoseStamped()
        msg.header.stamp = simulation.simulator.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = utils.scenic_point_to_ros_point(upd_pos)

        quaternion = utils.yaw_to_quaternion(normalizeAngle(self.dest.heading + math.pi/2))
        msg.pose.orientation = quaternion

        simulation.simulator.ego_goal_publisher.publish(msg)
        simulation.ads_internal_status = AdsInternalStatus.GOAL_SET
        print('Setting Ego destination done')

class SendEngageCommandAction(Action):
    def __init__(self):
        pass
    def canBeTakenBy(self, agent):
        return agent.isEgo

    def applyTo(self, obj, simulation):
        print("Sending engage command to activate the autonomous operation mode.")
        msg = Engage()
        msg.stamp = simulation.simulator.node.get_clock().now().to_msg()
        msg.engage = True

        simulation.simulator.ego_auto_engage_publisher.publish(msg)
        simulation.ads_internal_status = AdsInternalStatus.AUTONOMOUS_IN_PROGRESS
        simulation.publish_in_auto_mode_signal()

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
            simulation.simulator.ego_max_speed_publisher.publish(vel_limit_msg)

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
        simulation.simulator.follow_lane_publisher.publish(msg)

        # do a service request to confirm the command was sent and processed properly
        retry = 0
        req = DynamicControl.Request()
        req.json_request = msg.data
        while retry < 10:
            future = simulation.simulator.follow_lane_client.call_async(req)
            rclpy.spin_until_future_complete(simulation.simulator.node, future)
            response = future.result()
            if response.status.success:
                print(f"Sent follow lane command to {npc_obj.name} successfully.")
                break
            time.sleep(simulation.timestep)
            retry += 1

        if retry == 10:
            logtext = f"[ERROR] AWSIM failed to process follow lane action, "\
                      f"error message: {response.status.message}."
            print("[ERROR] " + logtext)
            raise RejectionException(logtext)

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
        print("[INFO] Original waypoints: ")
        for waypoint in self.waypoints:
            print(f"{waypoint.position}, heading: {waypoint.toHeading()}")
            corrected_position = simulation.simulator.network.correct_elevation(waypoint)
            ros_wps.append(utils.scenic_point_to_dict(corrected_position))

        print("[INFO] Corrected waypoints: ")
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
        simulation.simulator.follow_waypoints_publisher.publish(msg)

        # do a service request to confirm the command was sent and processed properly
        retry = 0
        req = DynamicControl.Request()
        req.json_request = msg.data
        while retry < 10:
            future = simulation.simulator.follow_waypoints_client.call_async(req)
            rclpy.spin_until_future_complete(simulation.simulator.node, future)
            response = future.result()
            if response.status.success:
                print(f"Sent follow waypoints command to {npc_obj.name} successfully.")
                break
            time.sleep(simulation.timestep)
            retry += 1

        if retry == 10:
            print(f"[ERROR] AWSIM failed to process follow waypoints action, "
                  f"error message: {response.status.message}.")
            raise RejectionException(logtext)
