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

npcStartPos = follow roadDirection from egoRightPos for 20
require npcStartPos in non_intersection_long_road_region

ego = new EgoCar at egoStartPos,
        with name "ego",
        with behavior AutonomousDrive(egoDestination, max_speed=11)

npc1 = new Car at npcStartPos,
        with name "npc1",
        with behavior FollowLaneWithDelay(threshold=15, target_speed=3)

terminate when ego.arrived_destination() or real_time_from_start() > 18