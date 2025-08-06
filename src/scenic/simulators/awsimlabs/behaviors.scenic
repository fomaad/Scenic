"""Behaviors for dynamic agents in AWSIM-Labs simulator."""

from scenic.simulators.awsimlabs.actions import *

behavior AutonomousDrive(target, max_speed=None):
    while not ego.localization_succeeded():
        wait
    time.sleep(1)
    take SetDestinationAction(target)

    while not ego.autonomous_mode_ready():
        wait
    time.sleep(1)
    take SendEngageCommandAction()

    set_speed_action = SetMaxSpeedAction(max_speed)
    while True:
        take set_speed_action

behavior FollowLane(target_speed=None, acceleration=None, deceleration=None):
    # if target_speed is None, it will follow the speed limit of the current lane
    while not ego.is_in_autonomous_mode():
        wait
    action = FollowLaneAction(target_speed, acceleration, deceleration)
    take action

behavior FollowWaypoints(waypoints, target_speed=None, acceleration=None, deceleration=None):
    while not ego.is_in_autonomous_mode():
        wait
    action = FollowWaypointsAction(waypoints, target_speed, acceleration, deceleration)
    take action

behavior FollowLaneWithDelay(threshold=15, target_speed=None, acceleration=None, deceleration=None):
    while not ego.is_in_autonomous_mode() or (distance from self to ego) > threshold:
        wait
    action = FollowLaneAction(target_speed, acceleration, deceleration)
    take action

behavior FollowWaypointsWithDelay(waypoints, threshold=15, target_speed=None, acceleration=None, deceleration=None):
    while not ego.is_in_autonomous_mode() or (distance from self to ego) > threshold:
        wait
    action = FollowWaypointsAction(waypoints, target_speed, acceleration, deceleration)
    take action

def cal_distance_delay(desired_distance_threshold, target_speed, distance_travel,
                       ego_speed, acceleration, npc_dis_to_mid_front):
    accel_time = target_speed/acceleration
    accel_dis = 0.5 * target_speed**2 / acceleration
    remain_dis = distance_travel - accel_dis
    remain_time = remain_dis / target_speed
    ego_travel_dis = ego_speed * (accel_time + remain_time)
    return desired_distance_threshold + ego_travel_dis - distance_travel + npc_dis_to_mid_front

behavior JamaCutinBehavior(waypoints, desired_distance_threshold, target_speed,
                                      ego_speed, acceleration=8.35, deceleration=None):
    distance_travel = distance from waypoints[0] to waypoints[1]
    threshold = cal_distance_delay(desired_distance_threshold, target_speed, distance_travel,
                                   ego_speed, acceleration, self.length*0.75)
    print(f"Threshold to trigger NPC moving: {threshold}")

    computation_delay=0.33
    threshold += computation_delay * ego_speed

    while not ego.is_in_autonomous_mode() or (distance from self to ego) > threshold:
        wait
    action = FollowWaypointsAction(waypoints, target_speed, acceleration, deceleration)
    take action