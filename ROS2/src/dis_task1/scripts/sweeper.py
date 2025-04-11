#!/usr/bin/env python3

import numpy as np
import cv2
from matplotlib import pyplot as plt
from shapely.geometry import Polygon, box, LineString
from scipy.spatial import Voronoi
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
import random
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, Quaternion

from dis_task1.msg import MoverMessage

class TraversalNode:
    def __init__(self, x, y):
        self.x : float = x
        self.y : float = y
        self.adjacent : list[TraversalNode] = []

    def position(self):
        return (self.x, self.y)

    def to_world(self, resolution, origin):
        x_world = origin[0] + self.x * resolution
        y_world = origin[1] + self.y * resolution
        return (x_world, y_world)

    def to_message(self, sweeper : Node, resolution, origin):
        mover_message = MoverMessage()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = sweeper.get_clock().now().to_msg()

        x, y = self.to_world(resolution, origin)
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.random_quat()

        mover_message.location = goal_pose
        mover_message.type = "sweep"
        mover_message.data = ""

        return mover_message

    def random_quat(self):
        quat_tf = quaternion_from_euler(0, 0, random.uniform(0, 2 * math.pi))

        # Convert a list to geometry_msgs.msg.Quaternion
        return Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])


class Sweeper(Node):
    def __init__(self):
        super().__init__("sweeper")

        self.map_client = self.create_client(
            GetMap,
            "/map_server/map"
        )

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info("Service /map_server/map established!")

    def pipeline(self, visualize=False):

        request = GetMap.Request()
        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Failed to call service /map_server/map')
            self.destroy_node()
            return
        
        grid : OccupancyGrid = future.result().map
        self.get_logger().info("Received map from service /map_server/map")

        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution

        grid_2d = np.array(grid.data).reshape((height, width))
        binary_map = np.where(grid_2d == 100, 1, 0)
        
        origin = (grid.info.origin.position.x, grid.info.origin.position.y)
        wall_points = self.extract_wall_points(binary_map, resolution, origin)
        wall_points = self.prune_close_points(wall_points)

        traversal_nodes = self.generate_traversal_nodes(binary_map, wall_points, resolution, origin)
        if visualize:
            self.visualize_map(binary_map, wall_points, traversal_nodes, resolution, origin)

        traversal_nodes = self.traverse_graph_dfs(traversal_nodes)
        return [node.to_message(self, resolution, origin) for node in traversal_nodes]


    def traverse_graph_dfs(self, start_nodes: list[TraversalNode]):
        visited = set()
        traversal_order = []

        def dfs(node : TraversalNode):
            if node in visited:
                return
            visited.add(node)
            traversal_order.append(node)
            for neighbor in node.adjacent:
                dfs(neighbor)

        for node in start_nodes:
            dfs(node)

        return traversal_order

    def extract_wall_points(self, binary_map, resolution, origin, spacing=0.1):

        search_map = np.uint8(binary_map * 255)
        contours, _ = cv2.findContours(search_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        wall_points = []
        for contour in contours:
            for i in range(len(contour)):
                if (i == len(contour) - 1):
                    direction = np.array(contour[0] - contour[i], dtype=np.float64)[0]
                else:
                    direction = np.array(contour[i + 1] - contour[i], dtype=np.float64)[0]
                direction_length = np.sqrt(np.sum(direction ** 2))
                direction /= direction_length

                px, py = contour[i][0]

                for dt in range(0, int(direction_length), int(spacing * 100)):
                    x = origin[0] + (dt * direction[0] + px) * resolution
                    y = origin[1] + (dt * direction[1] + py) * resolution
                    wall_points.append((x, y))

        return np.array(wall_points)

    def visualize_map(self, binary_map, wall_points, traversal_nodes, resolution, origin):

        points = self.world_to_pixel(wall_points, resolution, origin)
        
        plt.figure(figsize=(20, 20))
        plt.imshow(binary_map, cmap='gray', origin='upper')

        plt.scatter(points[:, 0], points[:, 1], c='red', s=1)

        for node in traversal_nodes:
            plt.scatter(node.x, node.y, c='red', s=15)
            for neighbor in node.adjacent:
                plt.plot([node.x, neighbor.x], [node.y, neighbor.y], 'y-', linewidth=1)

        plt.title("Navmesh Overlaid on Wall Map")
        plt.xlim(0, binary_map.shape[1])
        plt.ylim(0, binary_map.shape[0])
        plt.show()
            
    def generate_traversal_nodes(self, binary_map, wall_points, resolution, origin):
        
        if len(wall_points) < 4:
            raise ValueError("Too few wall points for Voronoi") # Realistically should never occur on maps we are using
        
        points = self.world_to_pixel(wall_points, resolution, origin)

        voronoi = Voronoi(points)
        polygons = self.filter_voronoi_cells(voronoi, binary_map.shape)
        edges = self.polygons_to_edges(polygons)
        edges = self.prune_edges(edges, binary_map)
        nodes : list[TraversalNode] = self.build_traversal_graph(edges)
        nodes = self.prune_close_nodes(nodes)
        self.prune_connections_over_walls(nodes, binary_map)
        nodes = self.remove_outside_node_clusters(nodes, binary_map.shape)
        return nodes

        

    def filter_voronoi_cells(self, voronoi, map_shape):
        height, width = map_shape
        map_bounds = box(1, 1, width - 1, height - 1)

        polygons = []
        for region_index in voronoi.point_region:
            region = voronoi.regions[region_index]
            if -1 in region or len(region) == 0:
                continue  # skip open or empty regions
            polygon = Polygon([voronoi.vertices[i] for i in region])
            clipped = polygon.intersection(map_bounds)

            minx, miny, maxx, maxy = clipped.bounds
            bbox_width = maxx - minx
            bbox_height = maxy - miny
            aspect_ratio = max(bbox_width / bbox_height, bbox_height / bbox_width)

            if 0 <= aspect_ratio <= 4:
                polygons.append(clipped)
        
        return polygons
    
    def world_to_pixel(self, world_coords, resolution, origin):
        x_points = [p[0] for p in world_coords]
        y_points = [p[1] for p in world_coords]

        x_pixels = [(x - origin[0]) / resolution for x in x_points]
        y_pixels = [(y - origin[1]) / resolution for y in y_points]
        return np.column_stack((x_pixels, y_pixels))

    def prune_close_points(self, points, min_distance=0.3):
        points = np.array(points)
        pruned = []

        for point in points:
            if len(pruned) == 0:
                pruned.append(point)
                continue
            
            distances = np.linalg.norm(np.array(pruned) - point, axis=1)
            if np.all(distances >= min_distance):
                pruned.append(point)

        return np.array(pruned)
    
    def polygons_to_edges(self, polygons):
        edges = []
        for polygon in polygons:
            exterior = polygon.exterior
            for i in range(len(exterior.coords) - 1):
                start = exterior.coords[i]
                end = exterior.coords[i + 1]
                edges.append([start, end])
        return np.array(edges)
    
    def is_edge_intersect_wall(self, edge, binary_map):
        line = LineString([edge[0], edge[1]])

        for point in line.coords:
            print(point)
            x, y = point
            if (0 <= int(x) < binary_map.shape[1]) and (0 <= int(y) < binary_map.shape[0]):
                if binary_map[int(y), int(x)] == 1:  # Wall found
                    return True
        return False

    def sample_points_on_edge(self, edge, spacing=1.0):
        x1, y1 = edge[0]
        x2, y2 = edge[1]
        
        length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        num_samples = int(np.ceil(length / spacing))
        
        x_values = np.linspace(x1, x2, num_samples)
        y_values = np.linspace(y1, y2, num_samples)
        
        points = [(x, y) for x, y in zip(x_values, y_values)]
        return points


    def is_edge_intersect_wall(self, edge, binary_map):
        sampled_points = self.sample_points_on_edge(edge)

        for (x, y) in sampled_points:
            x_idx = int(np.round(x))
            y_idx = int(np.round(y))

            if 0 <= x_idx < binary_map.shape[1] and 0 <= y_idx < binary_map.shape[0]:
                if binary_map[y_idx, x_idx] == 1:
                    return True
        
        return False

    def prune_edges(self, edges, binary_map):
        pruned_edges = []
        
        for edge in edges:
            if not self.is_edge_intersect_wall(edge, binary_map):
                pruned_edges.append(edge)
        
        return pruned_edges

    def build_traversal_graph(self, edges):
        node_map = {}

        def get_or_create_node(point):
            point = (point[0], point[1])
            if point not in node_map:
                node_map[point] = TraversalNode(*point)
            return node_map[point]

        for p1, p2 in edges:
            node1 = get_or_create_node(p1)
            node2 = get_or_create_node(p2)

            if node2 not in node1.adjacent:
                node1.adjacent.append(node2)
            if node1 not in node2.adjacent:
                node2.adjacent.append(node1)

        return list(node_map.values())

    def prune_close_nodes(self, nodes, min_distance=10):
        pruned_nodes = []
        removed = set()

        for i, node in enumerate(nodes):
            if node in removed:
                continue
            
            for other in nodes[i+1:]:
                if other in removed:
                    continue
                dx = node.x - other.x
                dy = node.y - other.y
                if (dx * dx + dy * dy) < (min_distance * min_distance):
                    removed.add(other)

                    for neighbor in other.adjacent:
                        if neighbor is not node and neighbor not in removed:
                            node.adjacent.append(neighbor)
                            neighbor.adjacent.append(node)
                    break

            pruned_nodes.append(node)

        for node in pruned_nodes:
            node.adjacent = [n for n in node.adjacent if n not in removed and n != node]

        return pruned_nodes

    def remove_outside_node_clusters(self, nodes, map_shape):
        map_width, map_height = map_shape
        def is_inside(node):
            return 0 <= node.x < map_width and 0 <= node.y < map_height

        visited = set()
        clusters_to_remove = []

        def dfs(node, cluster):
            visited.add(node)
            cluster.append(node)
            for neighbor in node.adjacent:
                if neighbor not in visited:
                    dfs(neighbor, cluster)

        for node in nodes:
            if node not in visited:
                cluster = []
                dfs(node, cluster)
                if all(not is_inside(n) for n in cluster) or len(cluster) < 10:
                    clusters_to_remove.extend(cluster)

        for node in clusters_to_remove:
            for neighbor in node.adjacent:
                neighbor.adjacent = [n for n in neighbor.adjacent if n != node]
            nodes.remove(node)

        return nodes
    
    def prune_connections_over_walls(self, nodes: list, binary_map: np.ndarray):
        height, width = binary_map.shape

        def is_wall(x, y):
            x = int(round(x))
            y = int(round(y))
            return not (0 <= x < width and 0 <= y < height) or binary_map[y, x] == 1

        def sample_points(p1, p2, spacing=1.0):
            x1, y1 = p1
            x2, y2 = p2
            dist = np.hypot(x2 - x1, y2 - y1)
            num_samples = max(2, int(dist / spacing))
            return [(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t) for t in np.linspace(0, 1, num_samples)]

        visited_pairs = set()
        for node in nodes:
            for neighbor in node.adjacent[:]:
                pair_id = tuple(sorted([id(node), id(neighbor)]))
                if pair_id in visited_pairs:
                    continue
                visited_pairs.add(pair_id)

                points = sample_points(node.position(), neighbor.position(), spacing=1.0)
                if any(is_wall(x, y) for x, y in points):
                    if neighbor in node.adjacent:
                        node.adjacent.remove(neighbor)
                    if node in neighbor.adjacent:
                        neighbor.adjacent.remove(node)

    

def main():
    rclpy.init(args=None)
    node = Sweeper()
    node.pipeline(visualize=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()