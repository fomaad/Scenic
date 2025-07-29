"""
to run:
scenic src/scenic/simulators/awsimlabs/scenario-test.scenic --2d --simulate
"""

param map = localPath('map_example/lanelet2_map.osm')

model scenic.simulators.awsimlabs.model

egoStartPos = new OrientedPoint in non_intersection_long_road_region, facing roadDirection
egoRightPos = new OrientedPoint right of egoStartPos by 3.5, facing roadDirection
require egoRightPos in non_intersection_long_road_region
require abs((roadDirection at egoStartPos).yaw - (roadDirection at egoRightPos).yaw) < 0.1

egoDestination = new OrientedPoint following roadDirection from egoStartPos for 80
require egoDestination in non_intersection_long_road_region

ego = new EgoCar at egoStartPos,
        with name "ego",
        with behavior AutonomousDrive(egoDestination, max_speed=11)

terminate when ego.arrived_destination()