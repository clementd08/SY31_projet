#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import numpy as np
import rospy
from math import cos, sin, atan2

class Costmap_node:
    def __init__(self):
        rospy.init_node("costmap node")
        rospy.loginfo("Starting costmap node...")

        # sub to the current position
        self.estimate_pose_subscriber = rospy.Subscriber("/estimate_pose", PoseStamped, self.receive_estimate_pose)
        self.current_pose = np.array([0., 0., 0.]) # x, y, theta

        # sub to the lidar
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.receive_lidar)


        # costmap
        self.costmap_size = 300  # points
        self.resolution = 0.01 # 1cm
        self.costmap = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)
        self.origin_x = 0
        self.origin_y = 0

        # distance max scan
        self.max_range = 0.4

        # timer callback update costmap
        update_costmap_period_s = 1
        rospy.Timer(rospy.Duration(update_costmap_period_s), self.update_costmap_callback)

        # pub the costmap
        self.costmap_publisher = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1)

        rospy.loginfo("Planning node started !")


    def receive_lidar(self, scan_msg):
        self.last_scan = scan_msg


    def receive_estimate_pose(self, msg):
        # get estimated position
        self.current_pose = np.array([msg.pose.position.x,
                                    msg.pose.position.y,
                                     atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2])


    def update_costmap_callback(self, _):
        scan = self.last_scan
        # print(len(scan.ranges))
        for i, distance in enumerate(scan.ranges):
            if distance > scan.range_min and distance < self.max_range:
                if (i>= 0 and i <=90) or (i>=270 and i<=360): 
                    # print(self.max_range)
                    # print(distance)
                    angle = scan.angle_min + i * scan.angle_increment
                    x = self.current_pose[0] + distance * cos(angle + self.current_pose[2])
                    y = self.current_pose[1] + distance * sin(angle + self.current_pose[2])
                    grid_x = int((x - self.origin_x) / self.resolution + self.costmap_size // 2)
                    grid_y = int((y - self.origin_y) / self.resolution + self.costmap_size // 2)

                    if 0 <= grid_x < self.costmap_size and 0 <= grid_y < self.costmap_size:
                        self.costmap[grid_y, grid_x] = 100  # Mark the cell as occupied


        # publish the costmap
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
    node = Costmap_node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass