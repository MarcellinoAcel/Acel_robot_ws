#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy.time
from std_msgs.msg import Int32MultiArray

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
import math
class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.subscription_button = self.create_subscription(
            Int32MultiArray,
            'button', 
            self.sign_callback,
            10)
        
        self.pub_pose = self.create_publisher(
            Pose2D,
            'robot_position',
            10)
        
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.robot_pose,
            10)

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.header.frame_id = 'map'

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def robot_pose(self,msg):
        pose = Pose2D()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        roll, pitch, yaw = self.quat_to_eular(msg.pose.pose.orientation)
        pose.theta = yaw
        self.pub_pose.publish(pose)
        self.get_logger().info(f" \nx={pose.x}\ny={pose.y}\nz={pose.theta}\n")

    def quat_to_eular(self, q):
        w = q.w
        x = q.x
        y = q.y
        z = q.z

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * x + y * z)
        if(abs(sinp) >=1):
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached with status: {0}'.format(result))
    

    def sign_callback(self, msg):
        button_A = msg.data[0]
        button_Y = msg.data[4]

        if button_A:
            self.send_goal(0.0, 0.0, 0.0)
        elif button_Y:
            self.send_goal(2.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception as e:
        node.get_logger().info(f'exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()