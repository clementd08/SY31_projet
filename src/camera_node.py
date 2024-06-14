#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, LaserScan

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('camera_node')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=1)

        
        self.last_distance = 1
        self.range_min = 0.25

        self.proche_mur = True

        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.distance_wall)
        
        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber_info = rospy.Subscriber('/image', Image, self.image_callback)

        # début tp4 (lancer camera launch)
        
        rospy.loginfo("camera node started !")


    def distance_wall(self, msg):
        self.last_distance = msg.ranges[0]

        # print("distance wall")
        # print(self.range_min)
        # print(self.last_distance)
        # print("\n")


    def image_callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img: Width*Height*3 Numpy matrix storing the image
        '''
        
        # print("\n")
        # min de pixel
        min_pixel = 500
        
        # Convert ROS Image -> OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # bgr to hsv
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # [rouge, vert, bleu]
        # masque bleue
        min_bleu = np.array([100, 50, 50])
        max_bleu = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, min_bleu, max_bleu)

        # nombre de pixels bleu
        nb_pixel_bleu = cv2.countNonZero(mask)
        # print("nombre de pixel bleu: ", nb_pixel_bleu)


        # masque rouge 
        min_rouge = np.array([0, 50, 50])
        max_rouge = np.array([10, 255, 255])

        mask = cv2.inRange(img_hsv, min_rouge, max_rouge)

        # nombre de pixels rouge
        nb_pixel_rouge = cv2.countNonZero(mask)
        # print("nombre de pixel rouge: ", nb_pixel_rouge)


        # selection du max de pixel
        if nb_pixel_rouge > nb_pixel_bleu : 
            fleche_rouge = True
        else :
            fleche_rouge = False
        
        if fleche_rouge:
            if nb_pixel_rouge > min_pixel :
                affichage = "Flèche Rouge, aller à droite"
        
        elif not fleche_rouge:
            if nb_pixel_bleu > min_pixel :
                affichage = "Flèche Bleu, aller à gauche"
        
        else : 
            affichage = "aucune fleche detecte"


        # print(self.last_distance)
        # print(self.range_min)
        # print("\n")

        if self.last_distance < self.range_min and self.last_distance !=0:
            if self.proche_mur :
                print(affichage)
                print("nombre de pixel bleu: ", nb_pixel_bleu)
                print("nombre de pixel rouge: ", nb_pixel_rouge)
                print(self.last_distance)
                print("\n")
            self.proche_mur = False
        
        else :
            self.proche_mur = True

            

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn(e)


if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

