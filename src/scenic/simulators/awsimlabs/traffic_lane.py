from enum import Enum
import numpy as np
from typing import Optional
import math

from scenic.simulators.awsimlabs import utils
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.type_support import *
from scenic.core.vectors import Vector

class TurnDirection(Enum):
    UNDEFINED = 0
    STRAIGHT = 1
    LEFT = 2
    RIGHT = 3

class LocationType(Enum):
    PRIVATE = 0
    URBAN = 1

class TrafficParticipant(Enum):
    UNDEFINED = 0
    VEHICLE = 1
    PEDESTRIAN = 2

# Represent a traffic lane
class TrafficLane:
    def __init__(self, id, turn_direction:TurnDirection, speed_limit: float,
                 left_coords, right_coords, location=LocationType.URBAN, participant=TrafficParticipant.UNDEFINED):
        """

        """
        self.id = id
        self.turn_direction = turn_direction
        self.speed_limit = speed_limit
        self.left_coords = left_coords
        self.right_coords = right_coords
        self.participant = participant

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
        return self.turn_direction != TurnDirection.UNDEFINED
    
    def get_2D_waypoints(self):
        return [(x,y) for (x,y,_) in self.way_points]
    
    def is_2dpoint_on_lane(self, point2d, heading: Optional[float]=None, heading_tolerance=0.15):
        """
        point2d can be a tuple (x,y)
        It is recommend to include the heading param (in radians)
        """
        if not self.polygon_2d_region.containsPoint(point2d):
            return False
        if heading is None:
            return True

        # check if the lane direction at $point2d same as the given $heading angle
        _, wp_id = self.correct_position(point2d)
        if wp_id == -1:
            # use WP0 -> WP1 as the lane direction
            wp_id = 0

        direction = self.way_points[wp_id + 1] - self.way_points[wp_id]
        yaw = math.atan2(direction[1], direction[0]) - math.pi/2
        print(f'yaw: {yaw}')
        if yaw < -math.pi:
            yaw += 2*math.pi
        print(f'yaw: {yaw}, heading: {heading}')
        return abs(heading - yaw) < heading_tolerance

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
        return Vector(px,py,0), -1

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