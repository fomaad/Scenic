from scenic.core.workspaces import *
from scenic.simulators.awsimlabs.network import *
from scenic.simulators.awsimlabs.simulator import AWSIMLabsSimulator
from scenic.simulators.awsimlabs.behaviors import *

network = load_map(globalParameters.map)
simulator AWSIMLabsSimulator(network)

road = network.road_2D_region()
workspace = Workspace(road)

intersection_region = network.intersection_2D_road_region()
non_intersection_region = network.non_intersection_2D_road_region()

roadDirection = network.road_direction

class AWSIMObject:
    """
    Abstract class for objects.
    """
    name: string
    isVehicle: bool

class Vehicle(AWSIMObject): 
    """
    Vehicle, either NPC or ego
    """
    name: ""
    regionContainedIn: road
    position: new Point on center_lane_lines
    parentOrientation: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    width: 2
    length: 4.5
    isEgo: False
    isVehicle: True

class Car(Vehicle):
    bodyStyle: Uniform("taxi", "hatchback", "smallCar", "truck", "van")

class EgoCar(Vehicle):
    isEgo: True
    width: 2.186
    length: 4.886
    name: "ego"

class Waypoint(OrientedPoint):
    heading: roadDirection at self.position
