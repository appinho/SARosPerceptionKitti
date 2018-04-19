#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SemanticSegmentation(object):
    def __init__(self):
        rospy.init_node('semantic_segmentation')

        self.sub = rospy.Subscriber('/kitti/camera_color_left/image_raw', 
            Image, self.callback_read_semantic_segmentation, queue_size=10)

        self.pub = rospy.Publisher('/pre_processing/segmented_semantic_image',
            Image, queue_size=10)

        self.bridge = CvBridge()

        self.message_counter = 0

        scenario = str(rospy.get_param('~scenario', '0001')).zfill(4)
        self.dir = '/home/simonappel/kitti_data/' + scenario \
            + '/segmented_semantic_images/'

        rospy.spin()

    def callback_read_semantic_segmentation(self, image):

        filename = self.dir + str(self.message_counter).zfill(10) + ".png"
        cv_image = cv2.imread(filename)

        try:
            if cv_image is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                rospy.loginfo("Semantic Segmentation Image published [%d]",
                    self.message_counter)
            else:        
                raise TypeError 
        except TypeError:
            rospy.logwarn("Semantic Segmentation Image not found!")

        self.message_counter += 1
        
if __name__ == '__main__':
    try: 
        SemanticSegmentation()
    except rospy.ROSInterruptException:
        pass