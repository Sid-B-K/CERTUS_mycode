#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
from heapq import heappush, heappop

class WaypointGenerator:
    def __init__(self):
        rospy.init_node('waypoint_generator')

        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 1.0)
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.1)
        self.min_grid_size = rospy.get_param('~min_grid_size', 10)
        self.ground_threshold = rospy.get_param('~ground_threshold', 0.2)
        self.obstacle_height = rospy.get_param('~obstacle_height', 0.3)
        self.safety_margin = rospy.get_param('~safety_margin', 0.5)

        self.grid_size = self.min_grid_size
        self.inflation_radius = int(round(self.safety_margin / self.grid_resolution))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ground_detected = False
        self.occupancy_grid = None
        self.grid_origin = None
        self.current_goal = None
        self.pointcloud_range = 0

        self.path_pub = rospy.Publisher('/waypoint_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1)
        self.grid_pub = rospy.Publisher('/path_planning_grid', OccupancyGrid, queue_size=1)

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.points_sub = rospy.Subscriber('/points_raw', PointCloud2, self.pointcloud_callback)

        rospy.loginfo("Waypoint Generator Initialized with dynamic grid sizing")

    def pointcloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                      msg.header.frame_id,
                                                      msg.header.stamp,
                                                      rospy.Duration(1.0))

            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            transformed_points = []
            ground_points = []
            obstacle_points = []

            max_distance = 0

            for point in points:
                x = point[0] + transform.transform.translation.x
                y = point[1] + transform.transform.translation.y
                z = point[2] + transform.transform.translation.z
                distance = math.sqrt(x**2 + y**2)
                if distance > max_distance:
                    max_distance = distance
                transformed_points.append((x, y, z))

                if z < self.ground_threshold:
                    ground_points.append((x, y, z))
                elif z > self.obstacle_height:
                    obstacle_points.append((x, y, z))

            if max_distance > 0:
                self.pointcloud_range = max_distance
                self.grid_size = max(2 * max_distance, self.min_grid_size)
                self.inflation_radius = int(round(self.safety_margin / self.grid_resolution))

            self.ground_detected = len(ground_points) > 100

            if self.ground_detected:
                self.update_occupancy_grid(transformed_points, obstacle_points)
                if self.current_goal is not None:
                    self.plan_path_to_goal(self.current_goal)

        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error in point cloud processing: %s", str(e))
        except Exception as e:
            rospy.logerr("Error in point cloud processing: %s", str(e))

    def update_occupancy_grid(self, all_points, obstacle_points):
        grid_cells = int(self.grid_size / self.grid_resolution)
        grid = np.zeros((grid_cells, grid_cells), dtype=np.int8)

        self.grid_origin = (-self.grid_size/2, -self.grid_size/2)

        for point in obstacle_points:
            x, y, _ = point
            grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
            grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)

            if 0 <= grid_x < grid_cells and 0 <= grid_y < grid_cells:
                for dx in range(-self.inflation_radius, self.inflation_radius+1):
                    for dy in range(-self.inflation_radius, self.inflation_radius+1):
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance <= self.inflation_radius:
                            nx, ny = grid_x + dx, grid_y + dy
                            if 0 <= nx < grid_cells and 0 <= ny < grid_cells:
                                grid[nx, ny] = 100

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "base_link"
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = grid_cells
        grid_msg.info.height = grid_cells
        grid_msg.info.origin.position.x = self.grid_origin[0]
        grid_msg.info.origin.position.y = self.grid_origin[1]
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = grid.flatten().tolist()

        self.occupancy_grid = grid
        self.grid_pub.publish(grid_msg)

    def goal_callback(self, msg):
        self.current_goal = msg
        if self.ground_detected:
            self.plan_path_to_goal(msg)
        else:
            rospy.logwarn("Ground not detected yet. Goal saved for when ground is detected.")

    def plan_path_to_goal(self, goal):
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                      goal.header.frame_id,
                                                      rospy.Time(0),
                                                      rospy.Duration(1.0))
            goal_in_base = do_transform_pose(goal, transform)

            goal_x = goal_in_base.pose.position.x
            goal_y = goal_in_base.pose.position.y
            goal_distance = math.sqrt(goal_x**2 + goal_y**2)

            if goal_distance > self.pointcloud_range * 0.9:
                rospy.logwarn("Goal is at %.2fm, beyond %.2fm point cloud range. Planning may be unreliable.", 
                             goal_distance, self.pointcloud_range)

            start = (int(self.grid_size/2 / self.grid_resolution), 
                     int(self.grid_size/2 / self.grid_resolution))
            goal_grid = (
                int((goal_x - self.grid_origin[0]) / self.grid_resolution),
                int((goal_y - self.grid_origin[1]) / self.grid_resolution)
            )

            grid_cells = int(self.grid_size / self.grid_resolution)
            if (goal_grid[0] < 0 or goal_grid[0] >= grid_cells or
                goal_grid[1] < 0 or goal_grid[1] >= grid_cells):
                rospy.logwarn("Goal is outside of planning grid (%.2fm). Trying to adjust grid.", self.grid_size)
                required_size = 2 * max(abs(goal_x), abs(goal_y))
                if required_size > self.grid_size:
                    self.grid_size = required_size
                    rospy.loginfo("Temporarily expanded grid to %.2fm for this planning cycle", self.grid_size)
                    self.update_occupancy_grid([], [])
                    goal_grid = (
                        int((goal_x - self.grid_origin[0]) / self.grid_resolution),
                        int((goal_y - self.grid_origin[1]) / self.grid_resolution)
                    )

            if (0 <= goal_grid[0] < grid_cells and 0 <= goal_grid[1] < grid_cells and
                self.occupancy_grid[goal_grid[0], goal_grid[1]] > 50):
                rospy.logwarn("Goal position is inside an obstacle!")
                return

            path_grid = self.a_star_search(start, goal_grid)

            if path_grid:
                simplified_path = self.simplify_path(path_grid)

                path = Path()
                path.header.stamp = rospy.Time.now()
                path.header.frame_id = "base_link"

                for i, point in enumerate(simplified_path):
                    world_x = point[0] * self.grid_resolution + self.grid_origin[0]
                    world_y = point[1] * self.grid_resolution + self.grid_origin[1]

                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x = world_x
                    pose.pose.position.y = world_y

                    if i < len(simplified_path)-1:
                        next_point = simplified_path[i+1]
                        next_x = next_point[0] * self.grid_resolution + self.grid_origin[0]
                        next_y = next_point[1] * self.grid_resolution + self.grid_origin[1]
                        yaw = math.atan2(next_y - world_y, next_x - world_x)
                    else:
                        yaw = math.atan2(goal_y - world_y, goal_x - world_x)

                    pose.pose.orientation = self.yaw_to_quaternion(yaw)
                    path.poses.append(pose)

                final_pose = PoseStamped()
                final_pose.header = path.header
                final_pose.pose = goal_in_base.pose
                path.poses.append(final_pose)

                self.path_pub.publish(path)
                self.publish_waypoint_markers(path)
                rospy.loginfo("Published obstacle-avoiding path with %d waypoints", len(path.poses))
            else:
                rospy.logwarn("A* failed to find path. Goal may be unreachable.")

            if self.pointcloud_range > 0:
                self.grid_size = max(2 * self.pointcloud_range, self.min_grid_size)

        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error: %s", str(e))
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

    def simplify_path(self, path):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        current_index = 0

        while current_index < len(path)-1:
            next_index = len(path)-1

            while next_index > current_index + 1:
                if not self.line_intersects_obstacle(path[current_index], path[next_index]):
                    break
                next_index -= 1

            simplified.append(path[next_index])
            current_index = next_index

        return simplified

    def line_intersects_obstacle(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if self.occupancy_grid[x, y] > 50:
                    return True
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if self.occupancy_grid[x, y] > 50:
                    return True
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        if self.occupancy_grid[x, y] > 50:
            return True

        return False

    def a_star_search(self, start, goal):
        if self.occupancy_grid is None:
            return None

        neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        heappush(oheap, (fscore[start], start))

        while oheap:
            current = heappop(oheap)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            close_set.add(current)

            for dx, dy in neighbors:
                neighbor = current[0] + dx, current[1] + dy
                grid_cells = int(self.grid_size / self.grid_resolution)
                if (neighbor[0] < 0 or neighbor[0] >= grid_cells or
                    neighbor[1] < 0 or neighbor[1] >= grid_cells):
                    continue
                if self.occupancy_grid[neighbor[0], neighbor[1]] > 50:
                    continue

                move_cost = 1.0 if dx == 0 or dy == 0 else 1.414

                tentative_g_score = gscore[current] + move_cost

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue

                if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))

        return None

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def publish_waypoint_markers(self, path):
        marker_array = MarkerArray()

        for i, pose in enumerate(path.poses):
            marker = Marker()
            marker.header = pose.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0 if i == len(path.poses)-1 else 0.0
            marker.color.g = 0.0 if i == len(path.poses)-1 else 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q

if __name__ == '__main__':
    try:
        node = WaypointGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
