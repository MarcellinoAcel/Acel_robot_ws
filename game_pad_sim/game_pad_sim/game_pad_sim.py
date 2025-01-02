import sys
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
import rclpy
from rclpy.node import Node
import time

class GamePad(Node):
    def __init__(self):
        super().__init__("game_pad_sim")
        
        self.publisher_axis = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_button = self.create_publisher(Int8MultiArray, 'button', 10)
        
        pygame.init()
        pygame.joystick.init()

        self.connect_joystick()

        self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")

        self.speed = 0.0
        self.button6_pressed = False
        self.button7_pressed = False

        self.create_timer(0.1, self.axis_callback)
        self.create_timer(0.1, self.button_callback)

    def connect_joystick(self):
        """Attempts to connect to the joystick. Retries if not available."""
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")
                break
            else:
                self.get_logger().warn("No joystick connected. Retrying in 2 seconds...")
                time.sleep(2)

    def check_joystick_connection(self):
        """Checks if the joystick is still connected and attempts to reconnect if necessary."""
        try:
            self.joystick.get_name()  # Will throw an exception if disconnected
        except pygame.error:
            self.get_logger().warn("Joystick disconnected. Attempting to reconnect...")
            self.connect_joystick()
            
    def button_callback(self):
        pygame.event.pump()

        msg = Int8MultiArray()
        button_states = []

        for i in range(self.joystick.get_numbuttons()):
            button_states.append(self.joystick.get_button(i))

        msg.data = button_states
        self.publisher_button.publish(msg)
        self.get_logger().info(f'\ncurrent_speed:{self.speed}\n')

    def axis_callback(self):
        pygame.event.pump()

        if self.joystick.get_button(6) and not self.button6_pressed:
            self.speed -= 1.0
            self.button6_pressed = True
        elif not self.joystick.get_button(6):
            self.button6_pressed = False

        if self.joystick.get_button(7) and not self.button7_pressed:
            self.speed += 1.0
            self.button7_pressed = True
        elif not self.joystick.get_button(7):
            self.button7_pressed = False

        if self.speed < 0.0:
            self.speed = 0.0

        linear_axis_X = -self.joystick.get_axis(1) * self.speed
        linear_axis_Y = -self.joystick.get_axis(0) * self.speed
        angular_axis_Z = -self.joystick.get_axis(2) * self.speed

        if abs(linear_axis_X) < 0.03:
            linear_axis_X = 0.0
        if abs(linear_axis_Y) < 0.03:
            linear_axis_Y = 0.0
        if abs(angular_axis_Z) < 0.03:
            angular_axis_Z = 0.0

        if linear_axis_X == 0.0 and linear_axis_Y == 0.0 and angular_axis_Z == 0.0:
            return 
            
        twist = Twist()
        twist.linear.x = linear_axis_X
        twist.linear.y = linear_axis_Y
        twist.angular.z = angular_axis_Z

        self.publisher_axis.publish(twist)

        self.get_logger().info(f"\nLinear Velocity X: {linear_axis_X}\nLinear Velocity Y: {linear_axis_Y}\nAngular Velocity: {angular_axis_Z}\n")
        self.get_logger().info(f"\nCurrent speed: {self.speed}")


def main(args=None):
    rclpy.init(args=args)

    game_pad = GamePad()

    rclpy.spin(game_pad)

    game_pad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
