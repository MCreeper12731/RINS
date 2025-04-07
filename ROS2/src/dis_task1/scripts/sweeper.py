#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy as np
from matplotlib import pyplot as plt
import cv2
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, box, LineString

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
        return TraversalNode(x_world, y_world)


class Sweeper(Node):
    def __init__(self):
        super().__init__("sweeper")

        self.map_client = self.create_client(
            GetMap,
            "/map_server/map"
        )
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            pass
        self.get_logger().info('Service /map_server/map established!')

        self.get_logger().info("Sweeper constructed!")

    def initialize_map(self):

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

        # self.visualize_map(binary_map, wall_points, resolution, origin)
        self.generate_and_plot_voronoi(binary_map, wall_points, resolution, origin)

    def extract_wall_points(self, binary_map, resolution, origin, spacing=0.1):

        search_map = np.uint8(binary_map * 255)
        contours, _ = cv2.findContours(search_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        wall_points = []
        for contour in contours:
            # self.print_np(contour)
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

    def visualize_map(self, binary_map, wall_points, resolution, origin):
        """ Plots the occupancy grid and overlays the extracted wall points using Matplotlib. """
        # Convert occupancy grid to displayable format (invert colors)

        # Flip vertically to match world coordinates
        display_map = np.flipud(binary_map)

        # Extract X, Y wall point coordinates for scatter plot
        x_points = [p[0] for p in wall_points]
        y_points = [p[1] for p in wall_points]

        # Convert world coordinates to pixel coordinates
        x_pixels = [(x - origin[0]) / resolution for x in x_points]
        y_pixels = [display_map.shape[0] - (y - origin[1]) / resolution for y in y_points]  # Flip Y axis

        # Plot the map
        plt.figure(figsize=(8, 8))
        plt.imshow(display_map, cmap='gray', origin='upper')
        plt.scatter(x_pixels, y_pixels, c='red', s=5, label="Wall Points")  # Overlay points
        plt.title("Occupancy Grid with Extracted Wall Points")
        plt.legend()
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.show()
            
    def generate_and_plot_voronoi(self, binary_map, wall_points, resolution, origin):
        # Get wall point coordinates (nonzero pixels)
        # wall_points = np.column_stack(np.where(binary_map == 1))
        
        if len(wall_points) < 4:
            raise ValueError("Too few wall points for Voronoi")

        # Reverse y and x for Voronoi (image coords: row = y, col = x)
        
        points = self.world_to_pixel(wall_points, resolution, origin, binary_map.shape[0])

        # Generate Voronoi diagram
        voronoi = Voronoi(points)
        polygons = self.filter_voronoi_cells(voronoi, binary_map.shape)
        edges = self.polygons_to_edges(polygons)
        edges = self.prune_edges(edges, binary_map)
        nodes : list[TraversalNode] = self.build_traversal_graph(edges)
        nodes = self.prune_close_nodes(nodes, binary_map.shape)
        nodes = self.remove_outside_node_clusters(nodes, binary_map.shape)

        # Plot
        plt.figure(figsize=(20, 20))
        plt.imshow(binary_map, cmap='gray', origin='upper')

        # for polygon in polygons:
        #     if polygon.geom_type == 'Polygon':
        #         x, y = polygon.exterior.xy
        #         plt.plot(x, y, color="blue", linewidth=1)

        # for edge in edges:
        #     plt.plot(edge[:, 0], edge[:, 1], color="red", linewidth=1)
        plt.scatter(points[:, 0], points[:, 1], c='red', s=1)

        

        for node in nodes:
            plt.scatter(node.x, node.y, c='red', s=15)
            for neighbor in node.adjacent:
                plt.plot([node.x, neighbor.x], [node.y, neighbor.y], 'y-', linewidth=1)

        plt.title("Navmesh Overlaid on Wall Map")
        plt.xlim(0, binary_map.shape[1])
        plt.ylim(0, binary_map.shape[0])
        plt.show()

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
    
    def world_to_pixel(self, world_coords, resolution, origin, map_height):
        x_points = [p[0] for p in world_coords]
        y_points = [p[1] for p in world_coords]

        # Convert world coordinates to pixel coordinates
        x_pixels = [(x - origin[0]) / resolution for x in x_points]
        y_pixels = [(y - origin[1]) / resolution for y in y_points]  # Flip Y axis
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
        
        # Calculate the total length of the line segment
        length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Determine the number of samples based on the spacing
        num_samples = int(np.ceil(length / spacing))
        
        # Generate evenly spaced points along the line segment
        x_values = np.linspace(x1, x2, num_samples)
        y_values = np.linspace(y1, y2, num_samples)
        
        points = [(x, y) for x, y in zip(x_values, y_values)]
        return points


    def is_edge_intersect_wall(self, edge, binary_map):
        # Sample points along the edge
        sampled_points = self.sample_points_on_edge(edge)

        # Check if any sampled point is inside a wall
        for (x, y) in sampled_points:
            # Round the coordinates to map indices, ensuring it's within map bounds
            x_idx = int(np.round(x))
            y_idx = int(np.round(y))

            # Check if the sampled point is inside a wall (binary map value 1)
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

    def prune_close_nodes(self, nodes, map_shape, min_distance=10):
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

        # Remove references to deleted nodes from adjacency lists
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

        # Remove nodes and fix adjacency
        for node in clusters_to_remove:
            for neighbor in node.adjacent:
                neighbor.adjacent = [n for n in neighbor.adjacent if n != node]
            nodes.remove(node)

        return nodes

    

def main():
    rclpy.init(args=None)
    node = Sweeper()
    node.initialize_map()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()