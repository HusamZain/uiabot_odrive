import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl
import odrive
from odrive.enums import *

class State0Config(Node):

    def __init__(self):
        super().__init__('state0_config')
        self.cli = self.create_client(AxisState, 'request_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service  not available, waiting again...')
        self.req0 = AxisState.Request()

    def send_request(self): 
            self.req0.axis = 0
            self.req0.state = 3
            self.future = self.cli.call_async(self.req0)

def main():
    rclpy.init()

    state0_config = State0Config()
    state0_config.send_request()

    while rclpy.ok():
        rclpy.spin_once(state0_config)
        if state0_config.future.done():
            try:
                response = state0_config.future.result()
            except Exception as e:
                state0_config.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                state0_config.get_logger().info(
                    'state 0 is configured ')                

            break

    state0_config.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
