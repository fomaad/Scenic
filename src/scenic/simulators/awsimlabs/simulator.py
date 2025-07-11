import scenic.core.simulators as simulators
import utils
from scenic.simulators.awsimlabs.network import *

import math, json
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_vehicle_msgs.msg import Engage
import std_msgs.msg

class AWSIMLabsSimulator(simulators.Simulator):
    def __init__(self, network:Network, *args, **kwargs):
        print("AWSIMLabsSimulator loading...")
        super().__init__(*args, **kwargs)
        # Initialize ROS2 node, publishers, subscribers, etc.
        rclpy.init()
        self.network = network
        self.node = rclpy.create_node('scenic_awsimlabs_interface')
        self.EgoPoseTopic = '/initialpose'
        self.EgoPosePublisher = self.node.create_publisher(
            PoseWithCovarianceStamped,
            self.EgoPoseTopic,
            10
        )

    def createSimulation(self, scene, **kwargs):
        return AWSIMLabsSimulation(scene, self, **kwargs)

class AWSIMLabsSimulation(simulators.Simulation):
    def __init__(self, scene, simulator, **kwargs):
        self.simulator = simulator

        super().__init__(scene, **kwargs)

    def step(self):
        # Send actions to AWSIM-Labs via ROS2
        # Wait for/receive new state
        # Update Scenic's internal state
        pass

    def getProperties(self, obj, properties):
        return {prop: getattr(obj, prop, None) for prop in properties}

    def destroy(self):
        # Clean up ROS2 node, etc.
        rclpy.shutdown()
        pass

    def createObjectInSimulator(self, obj):
        if obj.isEgo:
            self.create_ego_in_simulator(obj)
        else:
            self.spawn_npc_in_simulator(obj)
    
    def create_ego_in_simulator(self, ego_obj):
        print(f'Ego postion: {ego_obj.position}, heading: {ego_obj.heading}-{ego_obj.heading*180/math.pi}')
        _, upd_pos, _ = self.simulator.network.find_lane_and_correct_position(ego_obj.position)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.simulator.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = upd_pos.x
        msg.pose.pose.position.y = upd_pos.y
        msg.pose.pose.position.z = upd_pos.z

        # Convert heading (yaw) to quaternion
        # don't forget the +90deg
        quaternion = utils.yaw_to_quaternion(ego_obj.heading + math.pi/2)

        msg.pose.pose.orientation = quaternion
        self.simulator.EgoPosePublisher.publish(msg)
        rclpy.spin_once(self.simulator.node, timeout_sec=0.1)
        print("spawned Ego")

    def spawn_npc_in_simulator(self, npc):
        print(f'NPC postion: {npc.position}, heading {npc.heading}-{npc.heading*180/math.pi}')
        _, upd_pos, _ = self.simulator.network.find_lane_and_correct_position(npc.position)

        publisher = self.simulator.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/spawn',
            10
        )
        my_dict = {
            "name": npc.name,
            "body_style": npc.bodyStyle,
            "position": {
                "x": upd_pos.x,
                "y": upd_pos.y,
                "z": upd_pos.z
            },
            "orientation": {
                "x": npc.orientation.x,
                "y": npc.orientation.y,
                "z": npc.orientation.z,
                "w": npc.orientation.w,
            }
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        publisher.publish(msg)
        rclpy.spin_once(self.simulator.node, timeout_sec=0.1)
        print(f"spwaned NPC {npc.name}")

    def set_ego_destination(self, ego_obj, goal_position):
        _, upd_pos, _ = self.simulator.network.find_lane_and_correct_position(goal_position)

        publisher = self.simulator.node.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            10
        )
        msg = PoseStamped()
        msg.header.stamp = self.simulator.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = upd_pos.x
        msg.pose.position.y = upd_pos.y
        msg.pose.position.z = upd_pos.z

        quaternion = utils.yaw_to_quaternion(goal_position.yaw)
        msg.pose.orientation = quaternion

        publisher.publish(msg)
        rclpy.spin_once(self.simulator.node, timeout_sec=0.1)

    def send_engage_cmd(self, ego_obj):
        publisher = self.simulator.node.create_publisher(
            Engage,
            '/autoware/engage',
            10
        )
        msg = Engage()
        msg.stamp = self.simulator.node.get_clock().now().to_msg()
        msg.engage = True

        publisher.publish(msg)
        rclpy.spin_once(self.simulator.node, timeout_sec=0.1)