"""
to run: 
scenic src/scenic/simulators/awsimlabs/scenario-test2.scenic --2d --simulate
"""

param map = localPath('map_example/lanelet2_map.osm')

model scenic.simulators.awsimlabs.model

DISTANCE_THRESHOLD = 20

egoStartPos = new OrientedPoint in non_intersection_long_road_region, facing roadDirection
egoRightPos = new OrientedPoint right of egoStartPos by 3.5, facing roadDirection
require egoRightPos in non_intersection_long_road_region
require abs((roadDirection at egoStartPos).yaw - (roadDirection at egoRightPos).yaw) < 0.1

npcStartPos = follow roadDirection from egoRightPos for 30
require npcStartPos in non_intersection_long_road_region
require abs((roadDirection at egoStartPos).yaw - (roadDirection at npcStartPos).yaw) < 0.1

egoDestination = follow roadDirection from egoStartPos for 130
require egoDestination in long_road_region

npcWP0 = new Waypoint at npcStartPos
npcWP1 = new Waypoint following roadDirection from npcStartPos for 15
npcWP2 = new Waypoint following roadDirection from egoStartPos for 52
npcWP3 = new Waypoint following roadDirection from egoStartPos for 70
waypoints = [npcWP0, npcWP1, npcWP2, npcWP3]

for waypoint in [npcWP1, npcWP2]:
    require waypoint in long_road_region
    require abs((roadDirection at waypoint).yaw - (roadDirection at egoStartPos).yaw) < 0.1

ego = new EgoCar at egoStartPos,
        with name "ego",
        with behavior AutonomousDrive(egoDestination, max_speed=11)

npc1 = new Car at npcStartPos,
        with name "npc1",
        with body_style Uniform("taxi", "hatchback", "smallCar"),
        with behavior FollowWaypointsWithDelay(waypoints, threshold=DISTANCE_THRESHOLD, target_speed=3)

terminate when ego.arrived_destination() or real_time_from_start() > 70