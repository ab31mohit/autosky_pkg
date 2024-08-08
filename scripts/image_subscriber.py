#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import signal
import sys

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("/webcam1/image_raw", Image, self.image_callback)

        # Register signal handler
        signal.signal(signal.SIGINT, self.shutdown)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def shutdown(self, signum, frame):
        rospy.loginfo("Shutting down image subscriber node...\n")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("KeyboardInterrupt")
        sys.exit(0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        img_subscriber = ImageSubscriber()
        img_subscriber.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
