import sys
import xml.etree.ElementTree as ET
import math
from scenic.core.vectors import Orientation, VectorField
from scenic.core.distributions import *
from scenic.simulators.awsimlabs.traffic_lane import *

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
    
    def road_2D_region(self, include_private_roads=False):
        polygon_regions = []
        for lane in self.traffic_lanes:
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            polygon_regions.append(lane.polygon_2d_region)
        return PolygonalRegion.unionAll(polygon_regions)
    
    def road_2D_center_lines(self, include_private_roads=False):
        line_regions = []
        for lane in self.traffic_lanes:
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            line_regions.append(PolylineRegion(points=lane.get_2D_waypoints()))
        return PolylineRegion.unionAll(line_regions)

    def private_2D_roads(self):
        line_regions = []
        for lane in self.traffic_lanes:
            if lane.location == LocationType.PRIVATE:
                line_regions.append(PolylineRegion(points=lane.get_2D_waypoints()))
        return PolylineRegion.unionAll(line_regions)
    
    def intersection_2D_center_line_region(self, include_private_roads=False):
        line_regions = []
        for lane in self.traffic_lanes:
            if not lane.is_intersection_lane():
                continue
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            line_regions.append(PolylineRegion(points=lane.get_2D_waypoints()))
        return PolylineRegion.unionAll(line_regions)

    def non_intersection_2D_center_line_region(self, include_private_roads=False):
        line_regions = []
        for lane in self.traffic_lanes:
            if lane.is_intersection_lane():
                continue
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            line_regions.append(PolylineRegion(points=lane.get_2D_waypoints()))
        return PolylineRegion.unionAll(line_regions)
    
    def non_intersection_2D_road_region(self, include_private_roads=False):
        polygon_regions = []
        for lane in self.traffic_lanes:
            if lane.is_intersection_lane():
                continue
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            polygon_regions.append(lane.polygon_2d_region)
        return PolygonalRegion.unionAll(polygon_regions)

    def intersection_2D_road_region(self, include_private_roads=False):
        polygon_regions = []
        for lane in self.traffic_lanes:
            if not lane.is_intersection_lane():
                continue
            if not include_private_roads and lane.location == LocationType.PRIVATE:
                continue
            polygon_regions.append(lane.polygon_2d_region)
        return PolygonalRegion.unionAll(polygon_regions)

    def find_lane_for_point(self, point2d):
        for lane in self.traffic_lanes:
            if lane.is_2dpoint_on_lane(point2d):
                return lane
        return None
    
    def find_lane_and_correct_position(self, point):
        if isinstance(point, Vector):
            point2d = (point.x, point.y)
        elif isinstance(point, np.ndarray):
            point2d = (float(point[0]), float(point[1]))
        else:
            point2d = point
        lane = self.find_lane_for_point(point2d)
        if not lane:
            raise RejectionException(f"The position {point2d} is not inside any lane region.")
        point3d, wp_id = lane.correct_position(point2d)
        return lane, point3d, wp_id
    
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
                                         location_type))

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
        return TurnDirection.NULL
    elif turn_direction == "left":
        return TurnDirection.LEFT
    elif turn_direction == "right":
        return TurnDirection.RIGHT
    elif turn_direction == "straight":
        return TurnDirection.STRAIGHT
    return TurnDirection.NULL