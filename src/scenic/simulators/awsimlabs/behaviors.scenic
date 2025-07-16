"""Behaviors for dynamic agents in AWSIM-Labs simulator."""

from scenic.simulators.awsimlabs.actions import *

behavior AutonomousDrive(target):
    action = SetDestinationAction(target)
    take action

behavior FollowLane(target_speed=None, acceleration=None, deceleration=None):
    # if target_speed is None, it will follow the speed limit of the current lane
    action = FollowLaneAction(target_speed, acceleration, deceleration)
    take action

behavior FollowWaypoints(waypoints, target_speed=None, acceleration=None, deceleration=None):
    action = FollowWaypointsAction(waypoints, target_speed, acceleration, deceleration)
    take action