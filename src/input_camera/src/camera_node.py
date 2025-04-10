#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FramePublisher:
    def __init__(self):
        rospy.init_node('frame_publisher', anonymous=True)
        self.bridge = CvBridge()

        #subscribe to realsense camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image,self.image_callback)

        #Publisher to send frames to other nodes
        self.image_sub = rospy.Publisher("/processed/image", Image, queue_size=10)


    def image_callback(self, msg):
        try:
            #convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            cv2.imshow("Captured Frame",cv_image)
            cv2.waitKey(1)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            rospy.logerr(f"Error processing image : {e}")


if __name__ == "__main__":

    try:
        FramePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass