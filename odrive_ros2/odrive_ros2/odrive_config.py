import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl
import odrive
from odrive.enums import *

class OdriveConfig(Node):

    def __init__(self):
        super().__init__('odrive_config')
        self.cli = self.create_client(Trigger, 'connect_odrive')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 0 not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self): 
            self.future = self.cli.call_async(self.req)

def main():
    rclpy.init()

    odrive_config = OdriveConfig()
    odrive_config.send_request()

    while rclpy.ok():
        rclpy.spin_once(odrive_config)
        if odrive_config.future.done():
            try:
                response = odrive_config.future.result()
            except Exception as e:
                odrive_config.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                odrive_config.get_logger().info(
                    'odrive is connected ')                               
            break

    odrive_config.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
