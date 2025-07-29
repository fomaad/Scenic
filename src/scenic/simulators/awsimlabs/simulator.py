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

MOTION_STATE_UNKNOWN = 0
MOTION_STATE_STOPPED = 1
MOTION_STATE_STARTING = 2
MOTION_STATE_MOVING = 3

ROUTING_STATE_UNKNOWN = 0
ROUTING_STATE_UNSET = 1
ROUTING_STATE_SET = 2
ROUTING_STATE_ARRIVED = 3
ROUTING_STATE_CHANGING = 4

OPERATION_STATE_UNKNOWN = 0
OPERATION_STATE_STOP = 1
OPERATION_STATE_AUTONOMOUS = 2
OPERATION_STATE_LOCAL = 3
OPERATION_STATE_REMOTE = 4

class AWSIMLabsSimulator(simulators.Simulator):
    def __init__(self, network: Network, *args, **kwargs):
        print("AWSIMLabsSimulator loading...")
        super().__init__(*args, **kwargs)
        self.network = network

        # Initialize ROS2 node, publishers, subscribers, etc.
        rclpy.init()
        self.node = rclpy.create_node('scenic_awsimlabs_interface')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # ROS publishers
        # initial pose for Ego
        self.ego_pose_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )

        # autonomous engage command
        self.ego_auto_engage_publisher = self.node.create_publisher(
            Engage,
            '/autoware/engage',
            qos_profile
        )

        # Ego's goal setting
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
        # dynamically spawning NPCs
        self.dynamic_npc_spawning_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/spawn',
            qos_profile
        )

        # follow lane command
        self.follow_lane_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_lane',
            qos_profile
        )

        # follow waypoints command
        self.follow_waypoints_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_waypoints',
            qos_profile
        )

        self.npc_removing_publisher = self.node.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/removing',
            qos_profile
        )

        # service clients
        # ground truth kinematics
        self.gt_kinematics_client = self.node.create_client(
            GroundtruthKinematic,
            '/simulation/gt_srv/kinematic'
        )

        # localization initilization
        self.init_localization_request = self.node.create_client(
            InitializeLocalization,
            '/api/localization/initialize'
        )

        # execution state, include: motion state, routing state, operation state
        self.execution_state_client = self.node.create_client(
            ExecutionState,
            '/simulation/gt_srv/execution_state'
        )

        self.dynamic_npc_spawning_client = self.node.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/spawn_srv',
        )
        self.follow_lane_client = self.node.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/follow_lane_srv',
        )
        self.follow_waypoints_client = self.node.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/follow_waypoints_srv',
        )
        self.npc_removing_client = self.node.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/removing_srv',
        )
        self.ego_stopping_client = self.node.create_client(
            ChangeOperationMode,
            '/api/operation_mode/change_to_stop'
        )

    def createSimulation(self, scene, **kwargs):
        return AWSIMLabsSimulation(scene, self, **kwargs)

    def destroy(self):
        super().destroy()

        # Clean up ROS2 node, etc.
        self.node.destroy_node()
        rclpy.shutdown()

class AWSIMLabsSimulation(simulators.Simulation):
    def __init__(self, scene, simulator, **kwargs):
        print("\n\n[INFO] AWSIMLabsSimulation starting...")
        self.simulator = simulator
        self._destroyed = False
        self.ego_properties = {}
        self.vehicle_properties = {}
        self.ads_internal_status = AdsInternalStatus.UNINITIALIZED
        self.ego_motion_state = MOTION_STATE_STOPPED
        self.ego_init_pose = PoseWithCovariance()
        self.real_time = datetime.datetime.now().timestamp()
        self.real_start_time = self.real_time

        timestep = kwargs.pop('timestep', 0.1)
        super().__init__(scene, timestep=timestep, **kwargs)

    def step(self):
        if self._destroyed:
            return

        self.real_time = datetime.datetime.now().timestamp()

        # update object states
        self.gt_kinematics_request()
        self.upd_execution_state()

        time.sleep(self.timestep)

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
        print("AWSIMLabsSimulation cleaning...")
        super().destroy()
        if self._destroyed:
            raise RuntimeError("AWSIMLabsSimulation.destroy() called twice")
        self._destroyed = True

        if self.ego_motion_state is not MOTION_STATE_STOPPED and \
                self.ads_internal_status is not AdsInternalStatus.GOAL_ARRIVED:
            print("[INFO] Stopping Ego...")
            self.force_ego_stop()

        # remove NPC objects
        self.remove_npcs()

        print("[INFO] Cleaning up finished.")
        time.sleep(3)  # be careful with time.sleep

    def upd_execution_state(self):
        response = self.request_execution_state()

        self.ego_motion_state = response.motion_state
        if response.is_autonomous_mode_available and \
                self.ads_internal_status < AdsInternalStatus.AUTONOMOUS_MODE_READY:
            print("Autonomous operation mode is ready")
            self.ads_internal_status = AdsInternalStatus.AUTONOMOUS_MODE_READY

        if (response.routing_state == ROUTING_STATE_ARRIVED and
                self.ads_internal_status == AdsInternalStatus.AUTONOMOUS_IN_PROGRESS):
            print("Arrived destination")
            self.ads_internal_status = AdsInternalStatus.GOAL_ARRIVED

    def request_execution_state(self):
        while not self.simulator.execution_state_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Execution state ROS service not available, waiting...')

        # Create a request
        req = ExecutionState.Request()
        future = self.simulator.execution_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.simulator.node, future)
        return future.result()

    def createObjectInSimulator(self, obj):
        if obj.isEgo:
            self.create_ego_in_simulator(obj)
        else:
            self.spawn_npc_in_simulator(obj)

    def create_ego_in_simulator(self, ego_obj):
        print(f'Ego original postion: {ego_obj.position}, heading angle {ego_obj.heading * 180 / math.pi}')
        upd_pos = self.simulator.network.do_correct_elevation(ego_obj.position, ego_obj.heading)
        print(f'Ego corrected postion: {upd_pos}')

        # publish a pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.simulator.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        self.ego_init_pose.pose.position.x = upd_pos.x
        self.ego_init_pose.pose.position.y = upd_pos.y
        self.ego_init_pose.pose.position.z = upd_pos.z

        # Convert heading (yaw) to quaternion
        # don't forget to add 90deg
        quaternion = utils.yaw_to_quaternion(ego_obj.heading + math.pi / 2)
        print(f"Ego quaternion: {quaternion}")
        self.ego_init_pose.pose.orientation = quaternion

        cov = [0.0] * 36
        cov[0] = 0.25  # x
        cov[7] = 0.25  # y
        cov[35] = 0.01  # yaw
        self.ego_init_pose.covariance = cov
        msg.pose = self.ego_init_pose

        print(f"Sending initial pose msg: {msg}")

        self.simulator.ego_pose_publisher.publish(msg)

        if self.does_localization_succeed():
            print("[INFO] (Re-)Localization succeeded.")
            # do one more check about the ego position in the simulator
            self.gt_kinematics_request()

            retry = 0
            while retry < 10 and \
                    self.ego_properties['position'].distanceTo(Vector(upd_pos.x, upd_pos.y)) > 0.5:
                time.sleep(0.2)
                self.gt_kinematics_request()
                retry += 1

            if retry < 5:
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

    def spawn_npc_in_simulator(self, npc):
        print(f'NPC original postion: {npc.position}, heading angle: {npc.heading * 180 / math.pi}')
        upd_pos = self.simulator.network.do_correct_elevation(npc.position, npc.heading)
        print(f'NPC corrected postion: {upd_pos}')

        my_dict = {
            "name": npc.name,
            "body_style": npc.body_style,
            "position": {
                "x": upd_pos.x,
                "y": upd_pos.y,
                "z": upd_pos.z
            },
            # since AWSIM does not use orientation, it is not necessary to correct it
            "orientation": {
                "x": npc.orientation.x,
                "y": npc.orientation.y,
                "z": npc.orientation.z,
                "w": npc.orientation.w,
            }
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        self.simulator.dynamic_npc_spawning_publisher.publish(msg)

        # do a service request to confirm the spawning
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = self.simulator.dynamic_npc_spawning_client.call_async(req)
            rclpy.spin_until_future_complete(self.simulator.node, future)
            response = future.result()
            if response.status.success:
                print(f"Spawned NPC {npc.name}")
                break

            time.sleep(self.timestep)
            retry += 1

        if retry >= 10:
            logtext = f"Failed to spawn NPC vehicle, error message: {response.status.message}"
            print(f"[ERROR] {logtext}")
            raise RejectionException(logtext)

    def does_localization_succeed(self):
        # Wait for service to be available
        while not self.simulator.init_localization_request.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Localization initialization ROS service not available, waiting...')

        # Create a request
        req = InitializeLocalization.Request()
        req.pose.append(PoseWithCovarianceStamped())
        req.pose[0].header.stamp = self.simulator.node.get_clock().now().to_msg()
        req.pose[0].header.frame_id = 'map'
        req.pose[0].pose = self.ego_init_pose
        future = self.simulator.init_localization_request.call_async(req)
        rclpy.spin_until_future_complete(self.simulator.node, future)
        response = future.result()
        return response.status.success

    def gt_kinematics_request(self):
        while not self.simulator.gt_kinematics_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Ground truth kinematic ROS service not available, waiting...')

        # Create a request
        req = GroundtruthKinematic.Request()
        future = self.simulator.gt_kinematics_client.call_async(req)
        rclpy.spin_until_future_complete(self.simulator.node, future)
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

    def remove_npcs(self):
        for obj in self.objects:
            if obj.isVehicle and not obj.isEgo:
                self.remove_npc(obj)

    def remove_npc(self, npc):
        my_dict = {
            "target": npc.name,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        self.simulator.npc_removing_publisher.publish(msg)

        # do a service request to confirm the spawning
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = self.simulator.npc_removing_client.call_async(req)
            rclpy.spin_until_future_complete(self.simulator.node, future)
            response = future.result()
            if response.status.success:
                print(f"Removed NPC {npc.name}")
                break

            time.sleep(self.timestep)
            retry += 1

        if retry >= 10:
            logtext = f"Failed to remove NPC vehicle, error message: {response.status.message}"
            print(f"[ERROR] {logtext}")
            raise RejectionException(logtext)

    def real_time_from_start(self):
        return self.real_time - self.real_start_time

    def force_ego_stop(self):
        req = ChangeOperationMode.Request()
        future = self.simulator.ego_stopping_client.call_async(req)
        rclpy.spin_until_future_complete(self.simulator.node, future)
        response = future.result()
        if response.status.success:
            # stopping order was sent, now waiting for ego fully stop
            retry = 0
            while retry < 30:
                response = self.request_execution_state()
                if response.motion_state is MOTION_STATE_STOPPED:
                    print(f"Stopped Ego successfully.")
                    return True

                time.sleep(self.timestep)
                retry += 1
            print(f"Stop command was sent successfully, but the Ego seems not stopped.")
            return False
        else:
            print(f"[ERROR] Failed to stop Ego. Stop order was rejected.")
            return False