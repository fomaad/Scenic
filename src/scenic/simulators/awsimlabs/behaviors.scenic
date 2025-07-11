"""Behaviors for dynamic agents in AWSIM-Labs simulator."""

from scenic.simulators.awsimlabs.actions import *

behavior AutonomousDrive(target):
    action = SetDestinationAction(target)
    while True:
        take action