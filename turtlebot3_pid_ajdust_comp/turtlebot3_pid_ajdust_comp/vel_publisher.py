import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Turtlebot(Node):

    def __init__(self):
        super().__init__('pos_subscriber')
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        self.twist_tmp = Twist()
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(
        Odometry,
        'odom',
        self.listener_callback,
        10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        print(msg.pose.pose.position.x)
        print(msg.pose.pose.orientation.z)
        self.get_logger().info('Receiving turtlebot postion')


    def timer_callback(self):
        self.twist_tmp.linear.x += 0.1
        self.twist_tmp.angular.z += 0.1
        self.publisher_.publish(self.twist_tmp)
        self.get_logger().info('Publishing turtlebot velocity')
        self.i += 1

# class PosSubscriber(Node):
#
#     def __init__(self):
#         super().__init__('pos_subscriber')
#         self.subscription = self.create_subscription(
#             Odometry,
#             'odom',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning
#
#     def listener_callback(self, msg):
#         print(msg.pose.pose.position.x)
#         print(msg.pose.pose.orientation.z)
# class VelPublisher(Node):
#
#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0
#
#     def timer_callback(self):
#         msg = Twist()
#         msg.linear.x = float(self.i)
#         msg.angular.z = float(2*self.i)
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.i += 1


def main(args=None):
    rclpy.init(args=args)

    turtlebot = Turtlebot()


    rclpy.spin(turtlebot)

    turtlebot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
