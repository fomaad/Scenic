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