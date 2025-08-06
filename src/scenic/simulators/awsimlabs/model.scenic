from scenic.core.workspaces import *
from scenic.simulators.awsimlabs.network import *
from scenic.simulators.awsimlabs.simulator import AWSIMLabsSimulator
from scenic.simulators.awsimlabs.behaviors import *

network = load_map(globalParameters.map)
bind_along_lane_impl(network.along_lane_impl)
wait_until_trace_written = False if 'sim_wait_until_trace_written' not in globalParameters else globalParameters.sim_wait_until_trace_written
simulator AWSIMLabsSimulator(network, wait_until_trace_written)

vehicle_road = network.vehicle_2D_road_region()
center_lane_lines = network.vehicle_2D_road_center_lines()
workspace = Workspace(vehicle_road)
long_road_region = network.long_road_region()
non_intersection_long_road_region = network.non_intersection_long_road_region()

intersection_region = network.intersection_2D_road_region()
non_intersection_region = network.non_intersection_2D_road_region()

roadDirection = network.road_direction

class AWSIMObject:
    """
    Abstract class for objects.
    """
    name: string
    isVehicle: bool
    visibleDistance: 200

class Vehicle(AWSIMObject): 
    """
    Vehicle, either NPC or ego
    """
    name: ""
    regionContainedIn: vehicle_road
    position: new Point on center_lane_lines
    parentOrientation: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    width: 2
    length: 4.5
    isEgo: False
    isVehicle: True

class Car(Vehicle):
    body_style: Uniform("taxi", "hatchback", "smallCar", "truck", "van")

class EgoCar(Vehicle):
    isEgo: True
    width: 2.186
    length: 4.886
    name: "ego"

    def is_in_autonomous_mode(self) -> bool:
        """
        Check if the ego car is now in autonomous mode, heading to the goal
        """
        sim = simulation()
        return sim.ads_internal_status >= AdsInternalStatus.AUTONOMOUS_IN_PROGRESS

    def localization_succeeded(self) -> bool:
        """
        Check if the localization was succeeded
        """
        sim = simulation()
        return sim.ads_internal_status >= AdsInternalStatus.LOCALIZATION_SUCCEEDED

    def autonomous_mode_ready(self) -> bool:
        """
        Check if the autonomous mode is ready
        """
        sim = simulation()
        return sim.ads_internal_status >= AdsInternalStatus.AUTONOMOUS_MODE_READY

    def arrived_destination(self) -> bool:
        """
        Return true if the Ego arrived its destination
        """
        sim = simulation()
        return sim.ads_internal_status == AdsInternalStatus.GOAL_ARRIVED

class Waypoint(OrientedPoint):
    heading: roadDirection at self.position

def real_time_from_start():
    sim = simulation()
    return sim.real_time_from_start()