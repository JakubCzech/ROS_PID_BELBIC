import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class VelPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.i)
        msg.angular.z = float(2*self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    velPublisher = VelPublisher()

    rclpy.spin(velPublisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
