#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import numpy as np
import rospy
from math import cos, sin, atan2

class CostmapNode:
    def __init__(self):
        rospy.init_node("costmap_node")

        # Subscribers
        self.pose_subscriber = rospy.Subscriber("/estimate_pose", PoseStamped, self.receive_pose)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.receive_lidar)

        # Current pose
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta

        # Costmap parameters
        self.costmap_size = 300  # points
        self.resolution = 0.01  # 1 cm
        self.costmap = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)
        self.origin_x = 0
        self.origin_y = 0

        # Maximum scan range
        self.max_range = 0.4

        # Timer for updating costmap
        update_costmap_period_s = 1
        rospy.Timer(rospy.Duration(update_costmap_period_s), self.update_costmap)

        # Publisher
        self.costmap_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1)

        rospy.loginfo("Costmap node started!")

    def receive_lidar(self, scan_msg):
        self.last_scan = scan_msg

    def receive_pose(self, pose_msg):
        # Get estimated position
        self.current_pose = np.array([pose_msg.pose.position.x,
                                      pose_msg.pose.position.y,
                                      atan2(pose_msg.pose.orientation.z, pose_msg.pose.orientation.w) * 2])

    def update_costmap(self, _):
        scan = self.last_scan
        for i, distance in enumerate(scan.ranges):
            # condition pour ajouter seulement les points qui ne sont pas trop loin du robot
            if scan.range_min < distance < self.max_range:
                # condition sur les points à ajouter dans la costmap on ajoute seulement les points en face du robot pour éviter les répétitions
                if (0 <= i <= 90) or (270 <= i <= 360):
                    angle = scan.angle_min + i * scan.angle_increment
                    x = self.current_pose[0] + distance * cos(angle + self.current_pose[2])
                    y = self.current_pose[1] + distance * sin(angle + self.current_pose[2])
                    grid_x = int((x - self.origin_x) / self.resolution + self.costmap_size // 2)
                    grid_y = int((y - self.origin_y) / self.resolution + self.costmap_size // 2)

                    if 0 <= grid_x < self.costmap_size and 0 <= grid_y < self.costmap_size:
                        self.costmap[grid_y, grid_x] = 100  # Mark the cell as occupied

        # Publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "odom"
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap_size
        costmap_msg.info.height = self.costmap_size
        costmap_msg.info.origin.position.x = -self.costmap_size * self.resolution / 2
        costmap_msg.info.origin.position.y = -self.costmap_size * self.resolution / 2
        costmap_msg.info.origin.position.z = 0
        costmap_msg.info.origin.orientation.x = 0
        costmap_msg.info.origin.orientation.y = 0
        costmap_msg.info.origin.orientation.z = 0
        costmap_msg.info.origin.orientation.w = 1

        costmap_msg.data = self.costmap.flatten().tolist()
        self.costmap_publisher.publish(costmap_msg)

if __name__ == "__main__":
    node = CostmapNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
