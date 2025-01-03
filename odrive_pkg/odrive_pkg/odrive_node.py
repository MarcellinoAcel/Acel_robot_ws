
import odrive
from odrive.enums import *
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Pose2D
class OdriveControllerNode(Node):
    def __init__(self):
        super().__init__('odrive_controller_node')
        
        self.get_logger().info("Finding ODrive...")
        self.odrv0 = odrive.find_any()
        self.get_logger().info("ODrive found!")
        
        self.speed = 0.0

        self.odrv0.axis0.motor.config.pre_calibrated = True
        self.odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = self.speed
        
        self.odrv0.axis1.motor.config.pre_calibrated = True
        self.odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.controller.input_vel = self.speed

        self.get_logger().info(f"\nCurrent speed: {self.speed}")

        self.subscription_drive= self.create_subscription(Int8MultiArray, 'hats', self.button_callback,10)
        self.subscription_robot_pose = self.create_subscription(Pose2D, 'robot_position', self.robot_pose_callback,10)

        self.button12_pressed = False
        self.button13_pressed = False

        self.x_pose = 0
        self.y_pose = 0

    def robot_pose_callback(self, msg):
        self.x_pose = msg.x
        self.y_pose = msg.y
        
        distance = math.sqrt(math.pow(10.418 - msg.x) + math.pow(-0.764 - msg.y))
        
        v_total = distance * math.sqrt(9.81 / 2 * 1.43)
        
        angle_target = math.atan2(-0.764 - msg.y,10.418 - msg.x)

        self.get_logger().info(f"\ncurrent launcher speed ={v_total}\n")
        self.get_logger().info(f"\njarak target = {distance}\n")
        self.get_logger().info(f"\nangle_target = {angle_target}\n")

    def button_callback(self, msg):

        if msg.data[1] > 0 and not self.button12_pressed:
            self.speed += 10.0
            self.button12_pressed = True
        elif not msg.data[1]:
            self.button12_pressed = False

        if msg.data[1] < 0 and not self.button13_pressed:
            self.speed -= 10.0
            self.button13_pressed = True
        elif not msg.data[1]:
            self.button13_pressed = False

        self.speed = max(0.0, min(self.speed, 40.0))

        
        self.odrv0.axis1.controller.input_vel = self.speed
        self.odrv0.axis0.controller.input_vel = self.speed
        # self.get_logger().info(f"\nCurrent speed: {self.speed}")

def main(args=None):
    rclpy.init(args=args)
    node = OdriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping motors...")
        node.odrv0.axis0.controller.input_vel = 0
        node.odrv0.axis1.controller.input_vel = 0
        rclpy.shutdown()
    print('Hi from odrive_pkg.')


if __name__ == '__main__':
    main()
