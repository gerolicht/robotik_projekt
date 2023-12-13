"""
Simple driving node that is based on the driving behavior of a simple vacuum cleaner robot: The robot turns as long as
an obstacle is detected in the defined area, otherwise it drives straight ahead. To detect obstacles, only one measurement
value is used per scan of the laser scanner.
"""

import rclpy
import rclpy.node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleDriving(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('distance_to_turn', 0.5)
        self.declare_parameter('distance_to_slow_down', 1.0)
        self.declare_parameter('speed_fast', 0.15)
        self.declare_parameter('speed_slow', 0.1)
        self.declare_parameter('speed_turn', 0.4)
        self.declare_parameter('laserscan_beam_to_use', 0)

        # variable for the last sensor reading
        self.last_distance = 0.0

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received laser scan data
    def scanner_callback(self, msg):

        # saving the required sensor value, no further processing at this point
        self.last_distance = msg.ranges[self.get_parameter('laserscan_beam_to_use').get_parameter_value().integer_value]
        print(self.last_distance)


    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        distance_slow = self.get_parameter('distance_to_slow_down').get_parameter_value().double_value
        distance_turn = self.get_parameter('distance_to_turn').get_parameter_value().double_value
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value

        # no or far away obstacle
        if (self.last_distance == 0.0) or (self.last_distance > distance_slow):
            speed = self.get_parameter('speed_fast').get_parameter_value().double_value
            turn = 0.0
            print('drive fast', speed)

        # obstacle close enough to reduce speed
        elif (self.last_distance <= distance_slow) and (self.last_distance > distance_turn):
            speed = self.get_parameter('speed_slow').get_parameter_value().double_value
            turn = 0.0
            print('drive slow')

        # obstacle close enough to turn
        elif (self.last_distance <= distance_turn) and (self.last_distance > distance_stop):
            speed = 0.0
            turn = self.get_parameter('speed_turn').get_parameter_value().double_value
            print('turn')

        # obstacle too close, emergency stop
        else:
            speed = 0.0
            turn = 0.0
            print('stop')

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):

    print('Hi from robotik_projekt.')
    rclpy.init(args=args)

    node = SimpleDriving()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
