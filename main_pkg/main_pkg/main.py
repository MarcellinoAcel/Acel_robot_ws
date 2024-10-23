#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray

class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.subscription_button = self.create_subscription(Int32MultiArray,'button', self.sign_callback,10)
        self.get_logger().info('subscribed to /button')
        
    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta  # Set quaternion based on your needs
        goal_msg.pose.header.frame_id = 'map'

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
            self.get_logger().info("button 6 pressed, moving to {0, 0, 0}")
            self.send_goal(0.0, 0.0, 0.0)
        elif button_Y:
            self.get_logger().info("button Y pressed, moving to {2, 0, 0}")
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

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
