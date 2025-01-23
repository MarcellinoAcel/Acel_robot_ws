
import odrive
from odrive.enums import *
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int8
import math
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
        self.publisher_laser_indicator = self.create_publisher(Int8,"laser_indicator",10)
        self.publisher_current_speed = self.create_publisher(Int8, "launcher_speed", 10)

        self.button_up = False
        self.button_down = False

        self.x_pose = 0
        self.y_pose = 0

        self.v_total = 0
        self.w_launcher = 0
        self.angle_target = 0  
        self.distance = 0

    def robot_pose_callback(self, msg):
        self.x_pose = msg.x
        self.y_pose = msg.y
        
        self.distance = math.sqrt(math.pow(10.418 - msg.x,2) + math.pow(-0.764 - msg.y,2))
        
        self.v_total = self.distance * math.sqrt(9.81 / 2 * 1.43)
        self.w_launcher = self.v_total/0.06585
        self.angle_target = math.atan2(-0.764 - msg.y,10.418 - msg.x)


    def button_callback(self, msg):

        if msg.data[1] > 0 and not self.button_up:
            self.speed += 5.0
            self.button_up = True
        elif not msg.data[1]:
            self.button_up = False

        if msg.data[1] < 0 and not self.button_down:
            self.speed -= 5.0
            self.button_down = True
        elif not msg.data[1]:
            self.button_down = False
        
        self.speed = max(0.0, min(self.speed, 40.0))
        laser_ind_msg = Int8()
        laser_ind_msg.data = 1 if self.speed > 0 else 0

        self.publisher_laser_indicator.publish(laser_ind_msg)
        self.odrv0.axis1.controller.input_vel = self.speed
        self.odrv0.axis0.controller.input_vel = self.speed
        
        launcher_speed_msg = Int8()
        launcher_speed_msg.data = self.speed
        self.publisher_current_speed(launcher_speed_msg)
        # self.get_logger().info(f"\nCurrent speed: {self.speed}")
        self.get_logger().info(f"\n ball/launcher speed={self.v_total}/{self.w_launcher}\njarak target = {self.distance}\n angle_target = {self.angle_target}\nCurrent speed: {self.speed}\n")

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
