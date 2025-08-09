'''
To run this file using the Carla simulator:
    scenic examples/carla/pcla.scenic --2d --model scenic.simulators.carla.model --simulate
'''
param map = localPath('../../assets/maps/CARLA/Town02.xodr')
model scenic.simulators.carla.model

ego = new Car,
    with behavior NeatAutonomousDrivingBehavior()

# hard code
leadCar = new Car at (40, -109.4),
        with behavior FollowLaneBehavior(5.56)
