import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl
import odrive
from odrive.enums import *

class State0ConConfig(Node):

    def __init__(self):
        super().__init__('state0_con_config')
        self.cli = self.create_client(AxisState, 'request_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service  not available, waiting again...')
        self.req0 = AxisState.Request()
 
    def send_request(self):  
        self.req0.axis = 0
        self.req0.state = 8
        self.future = self.cli.call_async(self.req0)

def main():
    rclpy.init()

    state0_con_config = State0ConConfig()
    state0_con_config.send_request()

    while rclpy.ok():
        rclpy.spin_once(state0_con_config)
        if state0_con_config.future.done():
            try:
                response = state0_con_config.future.result()
            except Exception as e:
                state0_con_config.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                state0_con_config.get_logger().info(
                    'state 0 control is configured ')                

            break

    state0_con_config.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
