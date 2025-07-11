from enum import Enum
import numpy as np
from scenic.simulators.awsimlabs import utils
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import Vector

class TurnDirection(Enum):
    NULL = 0
    STRAIGHT = 1
    LEFT = 2
    RIGHT = 3

class LocationType(Enum):
    PRIVATE = 0
    URBAN = 1

# Represent a traffic lane
class TrafficLane:
    def __init__(self, id, turn_direction:TurnDirection, speed_limit: float,
                 left_coodrs, right_coords, location=LocationType.URBAN):
        """

        """
        self.id = id
        self.turn_direction = turn_direction
        self.speed_limit = speed_limit
        self.left_coords = left_coodrs
        self.right_coords = right_coords

        self.next_lanes = []
        self.prev_lanes = []
        # self.stop_line = None   TODO: parse this info
        # self.right_of_way_lanes = []   TODO: parse this info

        if not location:
            location = LocationType.URBAN
        self.location = location
        
        self.way_points = self.compute_waypoints()
        self.polygon_2d_region = self.compute_2D_polygon()

    def compute_waypoints(self):
        waypoints = []
        i = 0
        while i < len(self.left_coords) and i < len(self.right_coords):
            left_coord = np.array(self.left_coords[i])
            right_coord = np.array(self.right_coords[i])
            waypoints.append(
                (left_coord + right_coord)/2
            )
            i += 1
        if i < len(self.left_coords) or i < len(self.right_coords):
            # print(f'[ERROR] Lane #{self.id}: left_coords and right_coords have different lengths ({len(self.left_coords)} vs {len(self.right_coords)})')
            waypoints.append(
                (self.left_coords[-1] + self.right_coords[-1])/2
            )
        return waypoints
    
    def compute_2D_polygon(self):
        poly_coords = self.left_coords + self.right_coords[::-1]
        poly_2D_coords = [(x,y) for (x,y,_) in poly_coords]
        return PolygonalRegion(points = poly_2D_coords)
    
    def is_intersection_lane(self):
        return self.turn_direction != TurnDirection.NULL
    
    def get_2D_waypoints(self):
        return [(x,y) for (x,y,_) in self.way_points]
    
    def is_2dpoint_on_lane(self, point2d):
        """
        point2d can be a tuple (x,y)
        """
        return self.polygon_2d_region.containsPoint(point2d)
    
    def correct_position(self, point2d):
        """
        REQUIREMENT: point2d must be inside the lane.
        Return the point (Scenic Vector) on the center line of the lane by projecting point2d onto lane center line.
        The returned point must include the estimated z value.
        Also return the waypoint index of the starting point of the segment on which the point is located.
        """
        px,py = point2d
        waypoints = self.way_points
        for i in range(len(waypoints) - 1):
            proj, projection_inside_segment = utils.project_point_to_line_3d((px,py,0), waypoints[i], waypoints[i+1])
            if projection_inside_segment:
                return Vector(float(proj[0]), float(proj[1]), float(proj[2])), i    
        print(f'[ERROR] Cannot correct the postion {point2d} to the lane #{self.id} center line.')
        return None, -1

    def is_point_on_center_line(self, point, tolerance=1e-3):
        """
        :param tolerance:
        :param point: 2D
        :return: {True, i} if the point is on the lane center line;
                 where i is the waypoint index of the starting point of the segment
                 on which the point is located.
                 False otherwise.
        """
        px,py = point
        waypoints = self.way_points
        for i in range(len(waypoints) - 1):
            x1,y1,_ = waypoints[i]
            x2,y2,_ = waypoints[i+1]
            dis, projection_inside_segment = utils.distance_point_to_segment_2d(px,py, x1,y1, x2,y2)
            if projection_inside_segment and dis < tolerance:
                dis_point_wp1_2d = np.linalg.norm(np.array(point) - np.array((x1,y1)))
                dis_wps_2d = np.linalg.norm(np.array((x1,y1)) - np.array((x2,y2)))
                point3d = dis_point_wp1_2d/dis_wps_2d * (waypoints[i+1] - waypoints[i]) + waypoints[i]
                return True, i, point3d
        return False, -1, None

    def __str__(self):
        next_ids = [entry.id for entry in self.next_lanes]
        prev_ids = [entry.id for entry in self.prev_lanes]
        return f'Lane #{self.id}, next: {next_ids}, prev: {prev_ids}'

# for debugging
def plot_traffic_lanes(lanes, point1, point2):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 12))  # 1:2 height:width ratio

    for lane in lanes:
        # Unpack (x, y) from 3D tuples
        left_x, left_y = zip(*[(x, y) for x, y, _ in lane.left_coords])
        right_x, right_y = zip(*[(x, y) for x, y, _ in lane.right_coords])
        center_x, center_y = zip(*[(x, y) for x, y, _ in lane.way_points])

        ax.plot(left_x, left_y, 'r--', linewidth=1)
        ax.plot(right_x, right_y, 'b--', linewidth=1)
        ax.plot(center_x, center_y, 'g-', linewidth=1)
        ax.scatter(center_x, center_y, c='green', s=10, label=f'Lane {lane.id}')

    px1, py1 = point1
    px2, py2 = point2

    # plt.plot(point[0], point[1],'ro') 
    plt.plot(px1,py1,'x') 
    plt.plot(px2,py2,'ro') 

    ax.set_title("Traffic Lanes")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis('equal')
    ax.set_aspect(1)        # Set Y/X ratio
    ax.grid(True)

    # Optional: prevent duplicate labels in legend
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())

    plt.show()