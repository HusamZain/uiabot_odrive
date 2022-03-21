import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import pi


class TwistToMotors(Node):

    """
    TwistToMotors - converts a twist message to motor commands.  Needed for navigation stack
    """

    def __init__(self):
        #ini the node    
        super().__init__('twist_to_motors')                                   #node name
        self.nodename = "twist_to_motors"
        self.get_logger().info(f"-I- {self.nodename} started") 
        #declare robot'sparameters
        self.base_width = self.declare_parameter("base_width", 0.37).value
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.05).value
        self.gear_ratio = self.declare_parameter("gear_ratio", 20).value
        self.ticks_since_target = 0
        self.dx = 0     #robot linear speed
        self.dr = 0     #robot angular speed
        self.rate_hz = self.declare_parameter("rate_hz", 50).value

        #publisger - wheels angulare velocity
        self.pub_lmotor = self.create_publisher(Float32, 'lwheel_vtarget', 10)
        self.pub_rmotor = self.create_publisher(Float32, 'rwheel_vtarget', 10)
        #subscriptions
        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        self.create_timer(1.0/self.rate_hz, self.calculate_left_and_right_target)

    def calculate_left_and_right_target(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = Float32()
        left = Float32()
        
        right.data = ((self.dx + (self.dr * self.base_width)) / self.wheel_radius) / ( 2 * pi)  #odriver takes velocity command in turn/sec, so we should convert from rad to rev/sec = turn/sec
        left.data =  -1 * ((self.dx - (self.dr * self.base_width)) / self.wheel_radius) / ( 2 * pi)  #odriver takes velocity command in turn/sec, so we should convert from rad to rev/sec = turn/sec
        
        self.pub_lmotor.publish(left)
        self.pub_rmotor.publish(right)

        self.ticks_since_target += 1    

    def twist_callback(self, msg):
        self.dx = msg.linear.x
        self.dr = msg.angular.z


def main(args=None):
    rclpy.init(args=args)
    try:
        twist_to_motors = TwistToMotors()
        rclpy.spin(twist_to_motors)
    except rclpy.exceptions.ROSInterruptException:
        pass

    twist_to_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()