import datetime

import scenic.core.simulators as simulators
import utils
from scenic.simulators.awsimlabs.network import *
from scenic.simulators.awsimlabs.actions import AdsInternalStatus

import math, json, time, threading
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance
from autoware_vehicle_msgs.msg import Engage
from autoware_adapi_v1_msgs.msg import (VehicleKinematics,
                                        LocalizationInitializationState,
                                        OperationModeState,
                                        MotionState,
                                        RouteState)
from autoware_adapi_v1_msgs.srv import InitializeLocalization, ChangeOperationMode
from aw_monitor.srv import *
from tier4_planning_msgs.msg import VelocityLimit
import std_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class AWSIMLabsSimulator(simulators.Simulator):
    def __init__(self, network:Network, *args, **kwargs):
        print("AWSIMLabsSimulator loading...")
        super().__init__(*args, **kwargs)
        self.network = network

    def createSimulation(self, scene, **kwargs):
        return AWSIMLabsSimulation(scene, self, **kwargs)

class AWSIMLabsSimulation(simulators.Simulation):
    def __init__(self, scene, simulator, **kwargs):
        print("\n\n[INFO] AWSIMLabsSimulation starting...")
        self.simulator = simulator
        self._destroyed = False
        self.ego_properties = {}
        self.vehicle_properties = {}
        self.ads_internal_status = AdsInternalStatus.UNINITIALIZED
        self.ego_motion_state = MotionState.STOPPED
        self.ego_init_pose = PoseWithCovariance()

        self.real_time = datetime.datetime.now().timestamp()
        self.real_start_time = self.real_time

        self.localization_succeeded_time = float('inf')
        self.auto_mode_ready_time = float('inf')

        # Initialize ROS2 node, publishers, subscribers, etc.
        rclpy.init()
        self.node = rclpy.create_node('scenic_awsimlabs_interface')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # ROS publisher for initial pose for Ego
        self.ego_pose_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )
        self.init_localization_request = self.node.create_client(
            InitializeLocalization,
            '/api/localization/initialize'
        )

        # ROS publisher for autonomous engage command
        self.ego_auto_engage_publisher = self.node.create_publisher(
            Engage,
            '/autoware/engage',
            qos_profile
        )

        # ROS publisher for Ego goal setting
        self.ego_goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            qos_profile
        )

        # ROS publisher for max velocity setting
        self.ego_max_speed_publisher = self.node.create_publisher(
            VelocityLimit,
            '/planning/scenario_planning/max_velocity',
            qos_profile
        )

        # ROS publisher for dynamically spawning NPCs
        self.dynamic_npc_spawning_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/spawn',
            qos_profile
        )

        # ROS publisher for follow lane command
        self.follow_lane_cmd_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_lane',
            qos_profile
        )

        # ROS publisher for follow waypoints command
        self.follow_waypoints_cmd_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_waypoints',
            qos_profile
        )

        # service clients
        # ground truth kinematics
        self.gt_kinematics_client = self.node.create_client(
            GroundtruthKinematic,
            '/simulation/gt_srv/kinematic'
        )

        # subscribers
        # operation mode
        self.node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.operation_mode_callback,
            qos_profile
        )
        self.node.create_subscription(
            RouteState,
            '/api/routing/state',
            self.routing_state_callback,
            qos_profile
        )
        self.node.create_subscription(
            MotionState,
            '/api/motion/state',
            self.motion_state_callback,
            qos_profile
        )

        super().__init__(scene, **kwargs)
        # timestep = kwargs.pop('timestep', 0.1)
        # super().__init__(scene, timestep=timestep, **kwargs)

    def step(self):
        if (self._destroyed):
            return
        # update status of objects
        self.gt_kinematics_request()

        if self.ads_internal_status == AdsInternalStatus.UNINITIALIZED:
            if self.does_localization_succeed():
                self.ads_internal_status = AdsInternalStatus.LOCALIZATION_SUCCEEDED
                self.localization_succeeded_time = self.currentTime
        # elif self.ads_internal_status == AdsInternalStatus.GOAL_ARRIVED:
        #     print("Goal arrived, destroying simulation...")
        #     self.destroy()

        self.ros_spin(num=3)
        self.real_time = datetime.datetime.now().timestamp()

    def ros_spin(self, num=1):
        if (self._destroyed):
            return
        for i in range(num):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def getProperties(self, obj, properties):
        if obj.isEgo:
            values = self.ego_properties
            if values:
                return values
        elif obj.isVehicle:
            if obj.name in self.vehicle_properties:
                return self.vehicle_properties[obj.name]
        return {prop: getattr(obj, prop, None) for prop in properties}

    def destroy(self):
        print("AWSIMLabsSimulation destroying...")
        super().destroy()
        if self._destroyed:
            raise RuntimeError("AWSIMLabsSimulation.destroy() called twice")
        self._destroyed = True

        if self.ads_internal_status is not AdsInternalStatus.GOAL_ARRIVED:
            # stop vehicle
            stop_request = self.node.create_client(
                ChangeOperationMode,
                '/api/operation_mode/change_to_stop'
            )
            while not stop_request.wait_for_service(timeout_sec=10.0):
                print('[WARNING] Service /api/operation_mode/change_to_stop not available, waiting...')

            # Create a request
            req = ChangeOperationMode.Request()
            future = stop_request.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()
            if response.status.success:
                print("[INFO] Sent stop command, waiting for AV stop...")
                self.wait_until_veh_stop()
            else:
                logtext = f"Could not stop vehicle, error msg {response.status.message}"
                print(f'[ERROR] {logtext}')
                raise RejectionException(logtext)

        # Clean up ROS2 node, etc.
        self.node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Cleaning up finished.")
        time.sleep(3) # be careful with time.sleep

    def wait_until_veh_stop(self):
        while self.ego_motion_state is not MotionState.STOPPED:
            print("[INFO] Waiting for vehicle stop...")
            self.ros_spin()
            time.sleep(2) # be careful of using time.sleep

    def createObjectInSimulator(self, obj):
        if obj.isEgo:
            self.create_ego_in_simulator(obj)
        else:
            self.spawn_npc_in_simulator(obj)
    
    def create_ego_in_simulator(self, ego_obj):
        print(f'Ego original postion: {ego_obj.position}, heading angle {ego_obj.heading*180/math.pi}')
        upd_pos = self.simulator.network.do_correct_elevation(ego_obj.position, ego_obj.heading)
        print(f'Ego corrected postion: {upd_pos}')

        # publish a pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        self.ego_init_pose.pose.position.x = upd_pos.x
        self.ego_init_pose.pose.position.y = upd_pos.y
        self.ego_init_pose.pose.position.z = upd_pos.z

        # Convert heading (yaw) to quaternion
        # don't forget to add 90deg
        quaternion = utils.yaw_to_quaternion(ego_obj.heading + math.pi/2)
        print(f"Ego quaternion: {quaternion}")
        self.ego_init_pose.pose.orientation = quaternion

        cov = [0.0] * 36
        cov[0] = 0.25  # x
        cov[7] = 0.25  # y
        cov[35] = 0.03  # yaw
        self.ego_init_pose.covariance = cov
        msg.pose = self.ego_init_pose

        print(msg)

        self.ego_pose_publisher.publish(msg)

        if self.does_localization_succeed():
            print("[INFO] (Re-)Localization succeeded.")
            # do one more check about the ego position in the simulator
            self.gt_kinematics_request()
            # print("[DEBUG] Kinematic update succeeded.")
            if self.ego_properties['position'].distanceTo(Vector(upd_pos.x, upd_pos.y)) < 0.5:
                self.ads_internal_status = AdsInternalStatus.LOCALIZATION_SUCCEEDED
                print("Spawned Ego successfully!")
            else:
                logtext = "It seems that Ego vehicle in AWSIM-Labs simulator does not update its position."
                print(f"[ERROR] {logtext}")
                raise RejectionException(logtext)
        else:
            logtext = 'Ego localization failed after setting the initial pose'
            print(f"[ERROR] {logtext}")
            raise RejectionException(logtext)

    def does_localization_succeed(self):
        # Wait for service to be available
        while not self.init_localization_request.wait_for_service(timeout_sec=10.0):
            print('[WARNING] Localization initialization ROS service not available, waiting...')

        # Create a request
        req = InitializeLocalization.Request()
        req.pose.append(PoseWithCovarianceStamped())
        req.pose[0].header.stamp = self.node.get_clock().now().to_msg()
        req.pose[0].header.frame_id = 'map'
        req.pose[0].pose = self.ego_init_pose
        future = self.init_localization_request.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        return response.status.success

    def spawn_npc_in_simulator(self, npc):
        print(f'NPC original postion: {npc.position}, heading angle: {npc.heading}')
        upd_pos = self.simulator.network.do_correct_elevation(npc.position, npc.heading)
        print(f'NPC corrected postion: {upd_pos}')

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
        self.dynamic_npc_spawning_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        print(f"spwaned NPC {npc.name}")

    def gt_kinematics_request(self):
        while not self.gt_kinematics_client.wait_for_service(timeout_sec=10.0):
            print('[WARNING] Ground truth kinematic ROS service not available, waiting...')

        # Create a request
        req = GroundtruthKinematic.Request()
        future = self.gt_kinematics_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        self.ego_properties = self.extract_properties(response.groundtruth_ego)
        for vehicle in response.groundtruth_vehicles:
            self.vehicle_properties[vehicle.name] = self.extract_properties(vehicle)

    def extract_properties(self, kinematics):
        position = utils.ros2scenic_position(kinematics.pose.position)
        linear_velocity = utils.ros2scenic_position(kinematics.twist.linear)
        euler_angles = utils.ros2scenic_position(kinematics.pose.rotation)

        elevation = kinematics.pose.position.z
        heading = normalizeAngle(kinematics.pose.rotation.z - math.pi / 2)
        speed = math.hypot(*linear_velocity)
        angular_velocity = utils.ros2scenic_position(kinematics.twist.angular)
        angularSpeed = kinematics.twist.angular.z

        return dict(
            position=position,
            roll=euler_angles.x,
            pitch=euler_angles.y,
            yaw=heading,
            velocity=linear_velocity,
            speed=speed,
            angularVelocity=angular_velocity,
            angularSpeed=angularSpeed,
        )

    def operation_mode_callback(self, msg):
        if msg.is_autonomous_mode_available and \
        self.ads_internal_status < AdsInternalStatus.AUTONOMOUS_MODE_READY:
            print("Autonomous operation mode is ready")
            self.ads_internal_status = AdsInternalStatus.AUTONOMOUS_MODE_READY
            self.auto_mode_ready_time = self.currentTime

    def routing_state_callback(self, msg):
        if (msg.state == RouteState.ARRIVED and
                self.ads_internal_status == AdsInternalStatus.AUTONOMOUS_IN_PROGRESS):
            print("Arrived destination")
            self.ads_internal_status = AdsInternalStatus.GOAL_ARRIVED

    def motion_state_callback(self, msg):
        # print("[INFO] Received motion state msg: {}".format(msg))
        self.ego_motion_state = msg.state
        # if msg.state == MotionState.STOPPED:
        #     self.ego_motion_state = MotionState.STOPPED
