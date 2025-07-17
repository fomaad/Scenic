import sys
import xml.etree.ElementTree as ET
import math

from scenic.core.object_types import OrientedPoint, Point
from scenic.core.vectors import Orientation, VectorField
from scenic.core.distributions import *
from scenic.simulators.awsimlabs.traffic_lane import *

# some filter conditions
no_private_lane = lambda lane: lane.location != LocationType.PRIVATE
no_pedestrian_lane = lambda lane: lane.participant != TrafficParticipant.PEDESTRIAN

"""
There exist some lanes in the OSM map that are not visible in AWSIM-Labs simulator.
This filter function check if a given lane (in OSM map) is visible in AWSIM-Labs or not
"""
def is_visible_lane_in_awsimlabs(lane):
    poly_coords = lane.left_coords + lane.right_coords[::-1]
    for vertex in poly_coords:
        if vertex[0] < 81280 or vertex[0] > 81880:
            return False
        if vertex[1] < 49910 or vertex[1] > 50700:
            return False
    return True

class Network:
    def __init__(self, traffic_lanes):
        self.traffic_lanes = traffic_lanes
        self.road_direction = VectorField("road_direction", self.road_direction_at_point)

    def road_direction_at_point(self, point:Vector):
        """
        :param point: instance of scenic Vector
        :return:
        """
        lane, upd_point, wp_id = self.find_lane_and_correct_position((point.x,point.y))
        direction = lane.way_points[wp_id + 1] - lane.way_points[wp_id]
        yaw = math.atan2(direction[1], direction[0]) 
        # In Scenic, 0 is Oy(+) direction
        return Orientation._fromHeading(yaw - math.pi/2) 
    
    def get_traffic_lane(self, id):
        for entry in self.traffic_lanes:
            if entry.id == id:
                return entry
        return None

    def extract_lanes(self, *conditions):
        """
        *conditions: each one is a function lambda lane: ...
        """
        result = []
        for lane in self.traffic_lanes:
            if all(condition(lane) for condition in conditions):
                result.append(lane)
        return result

    def extract_2D_road(self):
        return self.extract_lanes( no_pedestrian_lane, no_private_lane, is_visible_lane_in_awsimlabs)

    def extract_non_intersection_2D_road(self):
        non_intersection = lambda lane: not lane.is_intersection_lane()
        return self.extract_lanes(no_pedestrian_lane, no_private_lane, is_visible_lane_in_awsimlabs, non_intersection)

    def extract_intersection_2D_road(self):
        intersection_only = lambda lane: lane.is_intersection_lane()
        return self.extract_lanes(no_pedestrian_lane, no_private_lane, is_visible_lane_in_awsimlabs, intersection_only)

    def road_2D_region(self):
        filtered_lanes = self.extract_2D_road()
        polygon_regions = [lane.polygon_2d_region for lane in filtered_lanes]
        return PolygonalRegion.unionAll(polygon_regions)

    def non_intersection_2D_road_region(self):
        filtered_lanes = self.extract_non_intersection_2D_road()
        polygon_regions = [lane.polygon_2d_region for lane in filtered_lanes]
        return PolygonalRegion.unionAll(polygon_regions)

    def intersection_2D_road_region(self):
        filtered_lanes = self.extract_intersection_2D_road()
        polygon_regions = [lane.polygon_2d_region for lane in filtered_lanes]
        return PolygonalRegion.unionAll(polygon_regions)

    def road_2D_center_lines(self):
        filtered_lanes = self.extract_2D_road()
        line_regions = [PolylineRegion(points=lane.get_2D_waypoints()) for lane in filtered_lanes]
        return PolylineRegion.unionAll(line_regions)
    
    def find_lane_for_point(self, point2d, heading: Optional[float]=None, heading_tolerance=0.174):
        """
        :param point2d:
        :param heading:
        """
        lanes = [lane for lane in self.traffic_lanes if lane.is_2dpoint_on_lane(point2d, heading, heading_tolerance)]
        if not lanes:
            print('No lane found')
            return None
        if len(lanes) > 1:
            lane_ids = [lane.id for lane in lanes]
            print(f'Found {len(lanes)} possible lanes ({lane_ids}) containing point {point2d}. '
                  f'By default, the lower-elevation lane was selected.')
            min_z = float('inf')
            result = None
            for lane in lanes:
                if lane.way_points[0][2] < min_z:
                    min_z = lane.way_points[0][2]
                    result = lane
            return result

        return lanes[0]

    def find_lane_and_correct_position(self, point, heading: Optional[float]=None, heading_tolerance=0.174):
        """
        mainly to correct the z-value (elevation) of the given 2D $point
        """
        vec = toVector(point)
        point2d = (vec.x, vec.y)
        lane = self.find_lane_for_point(point2d, heading, heading_tolerance)
        if not lane:
            raise RejectionException(f"The position {point2d} is not inside any lane region.")
        point3d, wp_id = lane.correct_position(point2d)
        return lane, point3d, wp_id

    def do_correct_elevation(self, point2d, heading: Optional[float]=None, heading_tolerance=0.174):
        _, point3d, _ = self.find_lane_and_correct_position(point2d, heading, heading_tolerance)
        return point3d

    def correct_elevation(self, point):
        if isinstance(point, OrientedPoint):
            return self.correct_elevation_for_orientedpoint(point)
        return self.do_correct_elevation(point)

    def correct_elevation_for_orientedpoint(self, orientedpoint: OrientedPoint, heading_tolerance=0.174):
        pos = orientedpoint.position
        return self.do_correct_elevation((pos.x,pos.y), orientedpoint.toHeading(), heading_tolerance)

    def precisely_find_lane_for_point(self, point2D, tolerance=1e-3):
        """
        DEPRECATED
        find lane on which the $point is located (on the center line)
        :param point: 2D
        :return:
         - lane: if found, None otherwise
         - wp_id: waypoint index of the starting point of the segment on which the point is located.
         - point3d: the input point with z value (in numpy array format)
        """
        for lane in self.traffic_lanes:
            is_on, wp_id, point3d = lane.is_point_on_center_line(point2D, tolerance)
            if is_on:
                return lane, wp_id, point3d
        return None, -1, None

# Parse given OSM map
def load_map(osm_file_path):
    tree = ET.parse(osm_file_path)
    root = tree.getroot()

    # Extract nodes
    node_map = {}
    for node in root.findall('node'):
        node_id = int(node.attrib['id'])
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in node.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags and 'ele' in tags:
            x = float(tags['local_x'])
            y = float(tags['local_y'])
            z = float(tags['ele'])
            node_map[node_id] = (x, y, z)

    # Extract ways
    way_map = {}
    for way in root.findall('way'):
        way_id = int(way.attrib['id'])
        nds = [int(nd.attrib['ref']) for nd in way.findall('nd')]
        way_map[way_id] = nds

    # Extract drivable lanelet relations
    lanes = []
    for relation in root.findall('relation'):
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in relation.findall('tag')}
        if tags.get('type') == 'lanelet' and tags.get('subtype') == 'road':
            speed_limit = float(tags.get('speed_limit'))
            turn_direction = parse_turn_direction(tags.get('turn_direction'))
            location_type_str = tags.get('location')
            location_type = LocationType.URBAN if not location_type_str or location_type_str=="urban" else LocationType.PRIVATE

            participant = TrafficParticipant.UNDEFINED
            if tags.get('participant:vehicle'):
                participant = TrafficParticipant.VEHICLE
            elif tags.get('participant:pedestrian'):
                participant = TrafficParticipant.PEDESTRIAN

            left = None
            right = None
            for member in relation.findall('member'):
                role = member.attrib.get('role')
                ref = int(member.attrib['ref'])
                if member.attrib['type'] == 'way':
                    if role == 'left':
                        left = ref
                    elif role == 'right':
                        right = ref
            if left in way_map and right in way_map:
                # Get coordinates for both sides
                left_coords  = [np.array(node_map[nid]) for nid in way_map[left] if nid in node_map]
                right_coords = [np.array(node_map[nid]) for nid in way_map[right] if nid in node_map]
                lanes.append(TrafficLane(int(relation.attrib['id']),
                                         turn_direction,
                                         speed_limit,
                                         left_coords,
                                         right_coords,
                                         location_type,
                                         participant))

    return Network(parse_lane_connection(lanes))

def parse_lane_connection(lanes):
    for i,lane in enumerate(lanes):
        for lane2 in lanes[i+1:]:
            if refer_to_same_point(
                lane.left_coords[0], lane.right_coords[0],
                lane2.left_coords[-1], lane2.right_coords[-1],
            ):
                lane.prev_lanes.append(lane2)
                lane2.next_lanes.append(lane)
            if refer_to_same_point(
                lane.left_coords[-1], lane.right_coords[-1],
                lane2.left_coords[0], lane2.right_coords[0],
            ):
                lane.next_lanes.append(lane2)
                lane2.prev_lanes.append(lane)
    return lanes

def refer_to_same_point(left_coord1, right_coord1, left_coord2, right_coord2):
    left_diff = np.linalg.norm(left_coord1 - left_coord2)
    right_diff = np.linalg.norm(right_coord1 - right_coord2)
    return left_diff <= 1e-3 and right_diff <= 1e-3

def parse_turn_direction(turn_direction):
    if not turn_direction:
        return TurnDirection.UNDEFINED
    elif turn_direction == "left":
        return TurnDirection.LEFT
    elif turn_direction == "right":
        return TurnDirection.RIGHT
    elif turn_direction == "straight":
        return TurnDirection.STRAIGHT
    return TurnDirection.UNDEFINED


# for debugging
def plot_traffic_lanes(lanes, point1=None, point2=None):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(24, 24))  # 1:2 height:width ratio

    for lane in lanes:
        # Unpack (x, y) from 3D tuples
        left_x, left_y = zip(*[(x, y) for x, y, _ in lane.left_coords])
        right_x, right_y = zip(*[(x, y) for x, y, _ in lane.right_coords])
        center_x, center_y = zip(*[(x, y) for x, y, _ in lane.way_points])

        # ax.plot(left_x, left_y, 'r--', linewidth=1)
        # ax.plot(right_x, right_y, 'b--', linewidth=1)
        ax.plot(center_x, center_y, 'g-', linewidth=2)
        ax.scatter(center_x, center_y, c='green', s=10, label=f'Lane {lane.id}')

    # px1, py1 = point1
    # px2, py2 = point2

    # plt.plot(point[0], point[1],'ro')
    # plt.plot(px1,py1,'x')
    # plt.plot(px2,py2,'ro')

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

if __name__ == '__main__':
    import os
    script_dir = os.path.dirname(__file__)
    map_path = os.path.join(script_dir, 'map_example/lanelet2_map.osm')
    network = load_map(map_path)
    plot_traffic_lanes(network.extract_2D_road())