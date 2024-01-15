"""
simple line following node
"""

import rclpy
import rclpy.node
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class LineFollowing(rclpy.node.Node):

    def __init__(self):
        super().__init__('line_following')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 90)
        self.declare_parameter('boundary_right', 200)
        self.declare_parameter('threshold_line', 100)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 0.3)

        # position of brightes pixel in
        self.lineposition = 0

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received image data
    def scanner_callback(self, data):

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]

        # get the lowest row from image
        img_row = img_gray[height-1,:]

        # show image
        cv2.imshow("IMG", img_gray)
        cv2.imshow("IMG_ROW", img_row)
        cv2.waitKey(1)


    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        # todo logic
        speed = 0.0
        turn = 0.0

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):

    print('Hi from robotik_projekt line following.')
    rclpy.init(args=args)

    node = LineFollowing()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
