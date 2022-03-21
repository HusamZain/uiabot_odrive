import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl
import odrive
from odrive.enums import *

class State1Config(Node):

    def __init__(self):
        super().__init__('state1_config')
        self.cli = self.create_client(AxisState, 'request_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service  not available, waiting again...')
        self.req1 = AxisState.Request()

    def send_request(self):  
        self.req1.axis = 1
        self.req1.state = 3
        self.future = self.cli.call_async(self.req1)
        
def main():
    rclpy.init()

    state1_config = State1Config()
    state1_config.send_request()

    while rclpy.ok():
        rclpy.spin_once(state1_config)
        if state1_config.future.done():
            try:
                response = state1_config.future.result()
            except Exception as e:
                state1_config.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                state1_config.get_logger().info(
                    'states 1 is configured ')
                               
            break

    state1_config.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
