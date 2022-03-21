#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32

NS_TO_SEC = 1000000000

class OdometryNode(Node):

        def __init__(self):
            
            super().__init__('odometry_node')                                   #node name
            self.nodename = "odometry_node"
            self.get_logger().info(f"-I- {self.nodename} started") 


            #### parameters #######
            self.rate_hz = self.declare_parameter("rate_hz", 10.0).value # the rate at which to publish the transform
            self.create_timer(1.0/self.rate_hz, self.update)
            self.base_width = float(self.declare_parameter('base_width', 0.37).value)  # The wheel base width in meters
            self.gear_ratio = float(self.declare_parameter('gear_ratio', 20).value)  # The gearbox ratio
            self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.05).value)  # The wheel radius in meter
            self.base_frame_id = self.declare_parameter('base_frame_id',
                                                    'base_link').value  # the name of the base frame of the robot
            self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

            #internal data
            self.angle_left = 0.0
            self.angle_right = 0.0  
            self.omega_left = 0.0
            self.omega_right = 0.0          
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0
            self.dx = 0.0
            self.dr = 0.0
            self.then = self.get_clock().now()

            #subscriptions
            self.create_subscription(Float32, "left_angle", self.lwheel_callback, 10)
            self.create_subscription(Float32, "right_angle", self.rwheel_callback, 10)
            self.create_subscription(Float32, "left_omega", self.lomega_callback, 10)
            self.create_subscription(Float32, "right_omega", self.romega_callback, 10)

            self.odom_pub = self.create_publisher(Odometry, "odom", 10)
            self.odom_broadcaster = TransformBroadcaster(self)

        def update(self):
            now = self.get_clock().now()
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.nanoseconds / NS_TO_SEC

            #calculate odometry
            
            # convert from turn to radian 
            #d_left = (-1*self.wheel_radius * self.angle_left / (2*pi)) / self.gear_ratio
            #d_right = (self.wheel_radius * self.angle_right / (2*pi)) / self.gear_ratio

            #forward kinematics
            self.dr = (self.wheel_radius/( 2* self.base_width))*(self.omega_right - self.omega_left)
            self.dx = 0.5 * self.wheel_radius * (self.omega_right + self.omega_left)
            #Compute odometry in typical way given the velocities of the robot
            delta_x = (self.dx * cos(self.th)) * elapsed
            delta_y = (self.dx * sin(self.th)) * elapsed
            delta_th = self.dr * elapsed

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            #since all odometry is 6dof we'll need a quatrenion created from yaw


            #d = (d_left + d_right) / 2 # distance traveled is the average of the two wheels

            #th = (d_right - d_left ) /self.base_width

            #calculate velocities
            #self.dx = d / elapsed
            #self.dr = th / elapsed

            #if d != 0:
                #calculate distance traveled in x and y
             #   x = cos(th) * d
              #  y = -sin(th) * d
                #calculate the final position of the robot
               # self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                #self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            #if th != 0:
             #   self.th = self.th + th

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)                

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            transform_stamped_msg.header.frame_id = self.odom_frame_id
            transform_stamped_msg.child_frame_id = self.base_frame_id
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation.x = quaternion.x
            transform_stamped_msg.transform.rotation.y = quaternion.y
            transform_stamped_msg.transform.rotation.z = quaternion.z
            transform_stamped_msg.transform.rotation.w = quaternion.w

            self.odom_broadcaster.sendTransform(transform_stamped_msg)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.dr
            self.odom_pub.publish(odom)


        def lwheel_callback(self, msg):
            self.angle_left = msg.data  

        def rwheel_callback(self,msg):
            self.angle_right = msg.data

        def lomega_callback(self,msg):
            self.omega_left = msg.data

        def romega_callback(self,msg):
            self.omega_right = msg.data

def main(args=None):
    rclpy.init(args=args)
    try:
        odometry_node = OdometryNode()
        rclpy.spin(odometry_node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

            #estimate wheel velocity 

            #omegaL = ( thetaL - thetaL_old_) / elapsed
            #omegaR = ( thetaR - thetaR_old_) / elapsed

            #forward kimematics

            #vth = (R / (2*b)) * (omegaR-omegaL)
            #vx = 0.5*R*(omegaR + omegaL)
            #vy = 0.0

            #print("omegaR")
            #print(omegaR)
            #print("omegaL:")
            #print(omegaL)

            # compute odometry in a typical way given the velocities of the robot
           # delta_x = (vx * cos(th)) * dt
            #delta_y = (vx * sin(th)) * dt
            #delta_th = vth * dt

            #x += delta_x
            #y += delta_y
            #th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf

            #odom_broadcaster.sendTransform(
             #   (x, y, 0.),
              #  odom_quat,
               # current_time,
                #"base_link",
                #"odom"
            #)

             # next, we'll publish the odometry message over ROS
            #odom = Odometry()
            #odom.header.stamp = current_time
            #odom.header.frame_id = "odom"

            # set the position
            #odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            #odom.child_frame_id = "base_link"
            #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))           
