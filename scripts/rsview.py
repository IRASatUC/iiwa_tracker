#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.cam_width = 640
        self.cam_height = 480
        self.dist = 0.0
        self.bridge=CvBridge()
        self.camera_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.caminfo_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)

    def depth_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # get the distance of center
        x, y = self.cam_width/2, self.cam_height/2
        self.dist = cv_img[x, y]

    def caminfo_callback(self, data):
        self.cam_width = data.width
        self.cam_height = data.height

    def draw_image(self, cv_img, title):
        color = (0,0,255)
        lineWeight = 2
        centerx, centery = self.cam_width/2, self.cam_height/2
        cv2.rectangle(cv_img, (centerx-50, centery-50), (centerx+50, centery+50), color, lineWeight)
        cv2.line(cv_img, (centerx-70, centery), (centerx+70, centery), color, lineWeight)
        cv2.line(cv_img, (centerx, centery-70), (centerx, centery+70), color, lineWeight)

        strDist = "The distance to zumopi is " + str(self.dist/1000.0) + " m"
        cv2.putText(cv_img, strDist, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow(title, cv_img)

    def color_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.draw_image(cv_img, "color")
        cv2.waitKey(3)


if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node("rsviewer", anonymous=True, log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destoryAllWindows();
