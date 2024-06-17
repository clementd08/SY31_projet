#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('camera_node')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=1)
        self.pub_cylinders = rospy.Publisher('/cylinders', MarkerArray, queue_size=10)

        # Constants
        self.last_distance = 1
        self.range_min = 0.25

        # Variables
        self.proche_mur = True
        self.id = 1 
        
        # color
        self.color_b = 1
        self.color_r = 1

        # Subscribers to the input topic. self.callback is called when a message is received
        self.subscriber_info = rospy.Subscriber('/image', Image, self.image_callback)
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.distance_wall)

        rospy.loginfo("camera node started !")



    def distance_wall(self, msg):
        # distance en face du robot(obtenue avec le lidar)
        self.last_distance = msg.ranges[0]


    def image_callback(self, msg):
        # Convert ROS Image -> OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # bgr to hsv
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # [red, green, blue]
        # masque bleue
        min_bleu = np.array([100, 50, 50])
        max_bleu = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, min_bleu, max_bleu)

        # nombre de pixels bleu
        nb_pixel_bleu = cv2.countNonZero(mask)

        # masque rouge 
        min_rouge = np.array([0, 50, 50])
        max_rouge = np.array([10, 255, 255])
        mask = cv2.inRange(img_hsv, min_rouge, max_rouge)

        # nombre de pixels rouge
        nb_pixel_rouge = cv2.countNonZero(mask)

        # selection du max de pixel et modifications des paramètres
        # On va regarder à chaque instant si on a plus de pixel bleu ou rouge mais on publie seulement à des moments spécifiques
        if nb_pixel_rouge > nb_pixel_bleu : 
            self.color_r = 1
            self.color_b = 0
            affichage = "Flèche Rouge, aller à droite"

        else :
            self.color_r = 0
            self.color_b = 1
            affichage = "Flèche Bleu, aller à gauche"


        # publication des messages sous condition
        # on met des conditins pour que le message soit publié une seule fois lorsqu'on arrive proche du mur
        if self.last_distance < self.range_min and self.last_distance !=0:
            if self.proche_mur :
                print(f"{affichage} \n")
                # publication sur la map de cylindre de la couleur de la flèche pour indiquer la direction
                self.cylinder_callback(msg)
            self.proche_mur = False
        
        else :
            self.proche_mur = True


        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn(e)

    
    def cylinder_callback(self, msg):
        # publication sur la map de cylindre de la couleur de la flèche pour indiquer la direction
        cylinders = MarkerArray()
        # paramètre pour définir les cylindres
        radius = 0.05

        cylinder = Marker()
        cylinder.header = msg.header
        cylinder.id = self.id
        self.id += 1
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose.position = Point(0, 0, 0)
        cylinder.pose.orientation.w = 1
        cylinder.scale.x = cylinder.scale.y = 2 * radius
        cylinder.scale.z = 0.3
        cylinder.color.r = self.color_r
        cylinder.color.g = 0
        cylinder.color.b = self.color_b
        cylinder.color.a = 0.5
        cylinder.lifetime = rospy.Duration(600)
        cylinders.markers.append(cylinder)

        self.pub_cylinders.publish(cylinders)
        


if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
