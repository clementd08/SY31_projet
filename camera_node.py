#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('camera node starting')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=1)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber_info = rospy.Subscriber('/turtlebotcam/camera_info', CameraInfo, self.callback_cam_info)
        
        rospy.loginfo("carmera node started !")


    def image_callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img: Width*Height*3 Numpy matrix storing the image
        '''
        
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
        min_bleu = np.array([0, 0, 255])
        max_bleu = np.array([125, 125, 255])
        mask = cv2.inRange(img_hsv, min_bleu, max_bleu)

        # nombre de pixels bleu
        nb_pixel_bleu = cv2.countNonZero(mask)
        print("nombre de pixel bleu: ", nb_pixel_bleu)


        # masque rouge 
        min_rouge = np.array([150, 0, 0])
        max_rouge = np.array([255, 125, 125])

        mask = cv2.inRange(img_hsv, min_rouge, max_rouge)

        # nombre de pixels rouge
        nb_pixel_rouge = cv2.countNonZero(mask)
        print("nombre de pixel rouge: ", nb_pixel_rouge)


        # selection du max de pixel
        if nb_pixel_rouge > nb_pixel_bleu : 
            fleche_rouge = True
        else :
            fleche_rouge = False
        
        if fleche_rouge:
            if nb_pixel_rouge > min_pixel :
                print("Flèche Rouge, aller à gauche")
        
        elif fleche_rouge:
            if nb_pixel_bleu > min_pixel :
                print("Flèche Bleu, aller à droite")
        
        else : 
            print("aucune fleche détecté")

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
