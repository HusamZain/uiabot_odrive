import sys
from math import pi
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl
import odrive
from odrive.enums import *
from std_msgs.msg import Float32 , String


class VelNode(Node):
  
    def __init__(self):
        super().__init__('vel_node')
        self.nodename = "vel_node"
        self.get_logger().info(f"{self.nodename} started")

        #declare parameters
        self.declare_parameter('connection.timeout', 15, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description='ODrive connection timeout in seconds'))
        #self.declare_parameter('battery.max_voltage', 4.2 * 6, ParameterDescriptor(
         #   type=ParameterType.PARAMETER_DOUBLE, description='Max battery voltage'))
        #self.declare_parameter('battery.min_voltage', 3.2 * 6, ParameterDescriptor(
          #  type=ParameterType.PARAMETER_DOUBLE, description='Min battery voltage'))
        #self.declare_parameter('battery.topic', 'barrery_percentage', ParameterDescriptor(
         #   type=ParameterType.PARAMETER_STRING, description='Battery percentage publisher topic'))

        self.connect_odrive_publisher_ = self.create_publisher(String,'Odrive_connection', 10)
        self.connect_odrive_publisher_timer = self.create_timer(0.1,self.connect_odrive_publisher_callback)
        self.driver: odrive.fibre.remote_object.RemoteObject = None



    def is_driver_ready(self):
        if self.driver:
            try:
                if self.driver.user_config_loaded:
                    return True
                else:
                    self.get_logger().warn('ODrive user config not loaded')
                    return False
            except:
                self.get_logger().error('Unexpected error:', sys.exc_info()[0])
                return False
        else:
            self.get_logger().debug('ODrive not connected')
            return False


    def connect_odrive_publisher_callback(self):
        
        msg = String()
        msg.data = 'ODrive  ready'
        self.connect_odrive_publisher_.publish(msg)
        self.get_logger().info('Connecting to ODrive')
        self.driver = odrive.find_any(
                timeout=self.get_parameter(
                    'connection.timeout'
                ).get_parameter_value().integer_value)
        self.get_logger().info('ODrive connected')
        self.get_logger().info(f"Connected to {self.driver.serial_number}")
        


def main(args=None):
    print('Hi from odrive_ros2.')
    rclpy.init(args=args)
    try:
        vel_node = VelNode()
        rclpy.spin(vel_node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    vel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



