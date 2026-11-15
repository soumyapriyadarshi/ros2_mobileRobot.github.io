#!/usr/bin/env python3

import math
import heapq
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # Parameters: coordinates must be in the /map frame.
        self.declare_parameter('start_x', -8.0)
        self.declare_parameter('start_y', -8.0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('allow_unknown', False)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.obstacle_threshold = int(
            self.get_parameter('obstacle_threshold').value
        )
        self.allow_unknown = bool(
            self.get_parameter('allow_unknown').value
        )

        self.map_data = None
        self.map_info = None
        self.last_map_signature = None

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/astar_path',
            10
        )

        self.metrics_pub = self.create_publisher(
            String,
            '/astar_metrics',
            10
        )

        self.get_logger().info('A* Planner started.')
        self.get_logger().info(
            f'Start: ({self.start_x}, {self.start_y}) | '
            f'Goal: ({self.goal_x}, {self.goal_y})'
        )
        self.get_logger().info('Waiting for /map ...')

    def map_callback(self, msg):
        # Avoid recalculating if the map has not changed.
        signature = (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            len(msg.data),
            hash(bytes((value + 1) % 256 for value in msg.data))
        )

        self.map_info = msg.info
        self.map_data = msg.data

        if signature == self.last_map_signature:
            return

        self.last_map_signature = signature
        self.get_logger().info('Map received. Running A*...')
        self.plan_path(msg.header.frame_id)

    def world_to_grid(self, x, y):
        """Convert map-frame world coordinates into grid coordinates."""
        origin = self.map_info.origin

        q = origin.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        dx = x - origin.position.x
        dy = y - origin.position.y

        # Rotate world displacement into the map's local coordinates.
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        resolution = self.map_info.resolution

        gx = int(math.floor(local_x / resolution))
        gy = int(math.floor(local_y / resolution))

        return gx, gy

    def grid_to_world(self, gx, gy):
        """Return the center of a grid cell in the map frame."""
        origin = self.map_info.origin
        resolution = self.map_info.resolution

        local_x = (gx + 0.5) * resolution
        local_y = (gy + 0.5) * resolution

        q = origin.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        world_x = (
            origin.position.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )

        world_y = (
            origin.position.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )

        return world_x, world_y

    def is_valid(self, x, y):
        width = self.map_info.width
        height = self.map_info.height

        return 0 <= x < width and 0 <= y < height

    def is_free(self, x, y):
        if not self.is_valid(x, y):
            return False

        width = self.map_info.width
        index = y * width + x
        occupancy = self.map_data[index]

        if occupancy == -1:
            return self.allow_unknown

        return occupancy < self.obstacle_threshold

    def heuristic(self, a, b):
        # Octile distance for an 8-connected grid.
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        return (
            max(dx, dy)
            + (math.sqrt(2.0) - 1.0) * min(dx, dy)
        )

    def get_neighbors(self, current):
        x, y = current

        movements = [
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0))
        ]

        for dx, dy, movement_cost in movements:
            nx = x + dx
            ny = y + dy

            if not self.is_free(nx, ny):
                continue

            # Prevent diagonal movement through obstacle corners.
            if dx != 0 and dy != 0:
                if not self.is_free(x + dx, y):
                    continue
                if not self.is_free(x, y + dy):
                    continue

            index = ny * self.map_info.width + nx
            occupancy = self.map_data[index]

            # Slightly penalize travel through partially occupied cells.
            occupancy_penalty = max(0, occupancy) / 100.0
            cost = movement_cost * (1.0 + occupancy_penalty)

            yield (nx, ny), cost

    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(
            open_heap,
            (self.heuristic(start, goal), 0.0, start)
        )

        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()
        expanded_nodes = 0

        while open_heap:
            _, current_cost, current = heapq.heappop(open_heap)

            if current in closed_set:
                continue

            if current == goal:
                path = [current]

                while current in came_from:
                    current = came_from[current]
                    path.append(current)

                path.reverse()

                return path, expanded_nodes, g_score[goal]

            closed_set.add(current)
            expanded_nodes += 1

            for neighbor, movement_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_cost = (
                    g_score[current] + movement_cost
                )

                if tentative_cost < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_cost

                    f_score = (
                        tentative_cost
                        + self.heuristic(neighbor, goal)
                    )

                    heapq.heappush(
                        open_heap,
                        (f_score, tentative_cost, neighbor)
                    )

        return None, expanded_nodes, float('inf')

    def publish_path(self, grid_path, frame_id):
        path_msg = Path()
        path_msg.header.frame_id = frame_id or 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (gx, gy) in enumerate(grid_path):
            x, y = self.grid_to_world(gx, gy)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orient each pose toward the next path point.
            if i < len(grid_path) - 1:
                next_x, next_y = self.grid_to_world(
                    grid_path[i + 1][0],
                    grid_path[i + 1][1]
                )
                yaw = math.atan2(next_y - y, next_x - x)
            elif i > 0:
                prev_x, prev_y = self.grid_to_world(
                    grid_path[i - 1][0],
                    grid_path[i - 1][1]
                )
                yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def plan_path(self, frame_id):
        if self.map_info is None or self.map_data is None:
            return

        start = self.world_to_grid(self.start_x, self.start_y)
        goal = self.world_to_grid(self.goal_x, self.goal_y)

        if not self.is_valid(*start):
            self.get_logger().error(
                f'Start position {start} is outside the map.'
            )
            return

        if not self.is_valid(*goal):
            self.get_logger().error(
                f'Goal position {goal} is outside the map.'
            )
            return

        if not self.is_free(*start):
            self.get_logger().error(
                'Start cell is occupied or unknown. Check start coordinates.'
            )
            return

        if not self.is_free(*goal):
            self.get_logger().error(
                'Goal cell is occupied or unknown. Check goal coordinates.'
            )
            return

        start_time = time.perf_counter()

        grid_path, expanded, total_cost = self.astar(start, goal)

        planning_time = time.perf_counter() - start_time

        if grid_path is None:
            self.get_logger().error('A* could not find a path.')
            return

        self.publish_path(grid_path, frame_id)

        resolution = self.map_info.resolution
        path_length = 0.0

        for i in range(1, len(grid_path)):
            dx = grid_path[i][0] - grid_path[i - 1][0]
            dy = grid_path[i][1] - grid_path[i - 1][1]
            path_length += math.hypot(dx, dy) * resolution

        metrics = String()
        metrics.data = (
            f'algorithm=A*; '
            f'path_length_m={path_length:.3f}; '
            f'grid_cost={total_cost:.3f}; '
            f'path_cells={len(grid_path)}; '
            f'expanded_nodes={expanded}; '
            f'planning_time_ms={planning_time * 1000.0:.3f}'
        )

        self.metrics_pub.publish(metrics)

        self.get_logger().info(
            f'A* path found | '
            f'Length: {path_length:.3f} m | '
            f'Expanded nodes: {expanded} | '
            f'Time: {planning_time * 1000.0:.3f} ms'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AStarPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()