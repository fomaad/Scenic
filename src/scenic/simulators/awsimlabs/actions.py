from scenic.domains.driving.actions import *
import utils
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from autoware_vehicle_msgs.msg import Engage
import std_msgs.msg

class SetDestinationAction(Action):
    def __init__(self, dest):
        self.dest = dest

    def canBeTakenBy(self, agent):
        return True

    def applyTo(self, obj, sim):
        self.set_ego_destination(obj, sim)
        self.send_engage_cmd(obj, sim)

    def set_ego_destination(self, obj, simulation):
        upd_pos = simulation.simulator.network.correct_elevation(self.dest)

        publisher = simulation.simulator.node.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            10
        )
        msg = PoseStamped()
        msg.header.stamp = simulation.simulator.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = utils.scenic_point_to_ros_point(upd_pos)

        quaternion = utils.yaw_to_quaternion(self.dest.yaw)
        msg.pose.orientation = quaternion

        publisher.publish(msg)
        rclpy.spin_once(simulation.simulator.node, timeout_sec=0.1)

    def send_engage_cmd(self, ego_obj, simulation):
        publisher = simulation.simulator.node.create_publisher(
            Engage,
            '/autoware/engage',
            10
        )
        msg = Engage()
        msg.stamp = simulation.simulator.node.get_clock().now().to_msg()
        msg.engage = True

        publisher.publish(msg)
        rclpy.spin_once(simulation.simulator.node, timeout_sec=0.1)

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
        publisher = simulation.simulator.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_lane',
            10
        )
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
        publisher.publish(msg)
        rclpy.spin_once(simulation.simulator.node, timeout_sec=0.1)
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
        publisher = simulation.simulator.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_waypoints',
            10
        )
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        ros_wps = []
        for waypoint in self.waypoints:
            corrected_position = simulation.simulator.network.correct_elevation(waypoint.position)
            ros_wps.append(utils.scenic_point_to_dict(corrected_position))
        
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
        publisher.publish(msg)
        rclpy.spin_once(simulation.simulator.node, timeout_sec=0.1)
        print(f"Sent follow waypoints command to {npc_obj.name}")
