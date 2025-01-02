import sys
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8
import rclpy
from rclpy.node import Node
import time

class GamePad(Node):
    def __init__(self):
        super().__init__("game_pad")
        
        self.publisher_axis = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.publisher_button = self.create_publisher(Int8MultiArray, 'button', 10)
        self.publisher_micros = self.create_publisher(Int8,'button_micros',10)
        self.publisher_catcher = self.create_publisher(Int8,'button_catcher', 10)
        self.publisher_allbutton_micros = self.create_publisher(Int8, "allButton", 10)
        self.publisher_hats = self.create_publisher(Int8MultiArray,'hats', 10)
        self.publisher_push = self.create_publisher(Int8, "push2launch", 10)
        
        pygame.init()
        pygame.joystick.init()

        self.connect_joystick()
        self.sign = 35

        self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")

        self.speed = 0.0
        self.button6_pressed = False
        self.button7_pressed = False
        self.create_timer(0.1, self.axis_callback)
        self.create_timer(0.1, self.button_callback)
        self.create_timer(0.1, self.micro_callback)

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
        hat_msg=Int8MultiArray()
        hat_states = []
        button_states = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        
        for i in range(self.joystick.get_numhats()):
            x, y = self.joystick.get_hat(i)
            
            hat_states.append(x if -128 <= x <= 127 else 0)
            hat_states.append(y if -128 <= y <= 127 else 0)

        msg.data = button_states   
        hat_msg.data= hat_states 
        self.publisher_button.publish(msg)
        self.publisher_hats.publish(hat_msg)
        self.get_logger().info(f'\ncurrent speed : {self.speed}\n')
        self.get_logger().info(f'Publishing button states: {button_states}')
        self.get_logger().info(f"Hat States: {hat_states}")

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
            
        if abs(self.joystick.get_axis(0)) or abs(self.joystick.get_axis(1)) or abs(self.joystick.get_axis(0)):
            linear_axis_Y = self.joystick.get_axis(0) * self.speed
            linear_axis_X = -self.joystick.get_axis(1) * self.speed
            angular_axis_Z = self.joystick.get_axis(2) * self.speed
            
            
        if abs(self.joystick.get_axis(0)) < 0.06:
            linear_axis_Y = 0.0
        if abs(self.joystick.get_axis(1)) < 0.06:
            linear_axis_X = 0.0
        if abs(self.joystick.get_axis(2)) < 0.06:
            angular_axis_Z = 0.0
            
        if linear_axis_X == 0.0 and linear_axis_Y == 0.0 and angular_axis_Z == 0.0:
            return 
        
        twist = Twist()
        twist.linear.x = linear_axis_X
        twist.linear.y = linear_axis_Y
        twist.angular.z = angular_axis_Z

        self.publisher_axis.publish(twist)
        
        self.get_logger().info(f"\nLinear Velocity X: {linear_axis_X}\nLinear Velocity Y: {linear_axis_Y}\nAngular Velocity: {angular_axis_Z}\n")
        self.get_logger().info(f"\nCurrent speed: {self.joystick.get_axis}")


    def micro_callback(self):
        pygame.event.pump()

        msg = Int8()
        msg.data = self.joystick.get_button(9)
        catch = Int8()
        catch.data = self.joystick.get_button(8)
        allbutton = Int8()
        pressed = False 
        allbutton.data = self.sign
        push = Int8()
        push.data = self.joystick.get_button(11)
        for i in range(self.joystick.get_numbuttons()):
            if(self.joystick.get_button(i)):
                self.sign = i
                pressed = True
                break

        if not pressed:
            self.sign = 35
        self.publisher_allbutton_micros.publish(allbutton)
        self.publisher_catcher.publish(catch)
        self.publisher_push.publish(push)
        self.publisher_micros.publish(msg)
def main(args=None):
    rclpy.init(args=args)

    game_pad = GamePad()

    rclpy.spin(game_pad)

    game_pad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
