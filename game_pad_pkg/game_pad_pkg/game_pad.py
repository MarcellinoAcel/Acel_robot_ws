import sys
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node

class GamePad(Node):
    def __init__(self):
        super().__init__("game_pad")
        
        self.publisher_axis = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.publisher_button = self.create_publisher(Int32MultiArray, 'button', 10)
        
        # Inisialisasi pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick connected.")
            sys.exit(1)

        # Inisialisasi joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")

        # Inisialisasi atribut tambahan 
        self.speed = 0.0  # Set default speed
        self.button6_pressed = False
        self.button7_pressed = False

        # Timer untuk memanggil callback axis dan button
        self.create_timer(0.1, self.axis_callback)
        self.create_timer(0.1, self.button_callback)

    def button_callback(self):
        pygame.event.pump()

        msg = Int32MultiArray()
        button_states = []

        # Perbaikan akses tombol joystick
        for i in range(self.joystick.get_numbuttons()):
            button_states.append(self.joystick.get_button(i))  # Menggunakan get_button(i)

        msg.data = button_states
        self.publisher_button.publish(msg)

        self.get_logger().info(f'Publishing button states: {msg.data}')

    def axis_callback(self):
        pygame.event.pump()

        # Mengubah kecepatan dengan tombol 6 dan 7
        if self.joystick.get_button(6) and not self.button6_pressed:
            self.speed -= 0.2
            self.button6_pressed = True  # Pastikan tombol tidak tertekan berulang kali
        elif not self.joystick.get_button(6):
            self.button6_pressed = False

        if self.joystick.get_button(7) and not self.button7_pressed:
            self.speed += 0.2
            self.button7_pressed = True
        elif not self.joystick.get_button(7):
            self.button7_pressed = False

        # Batas minimum kecepatan
        if self.speed < 0.0:
            self.speed = 0.0

        # Mengambil nilai dari axis joystick
        linear_axis_Y = self.joystick.get_axis(0) * self.speed
        linear_axis_X = -self.joystick.get_axis(1) * self.speed
        angular_axis_Z = self.joystick.get_axis(2) * self.speed

        # Menghilangkan noise dari joystick
        if abs(linear_axis_X) < 0.03:
            linear_axis_X = 0.0
        if abs(linear_axis_Y) < 0.03:
            linear_axis_Y = 0.0
        if abs(angular_axis_Z) < 0.03:
            angular_axis_Z = 0.0

        # Membuat pesan Twist
        twist = Twist()
        twist.linear.x = linear_axis_X
        twist.linear.y = linear_axis_Y
        twist.angular.z = angular_axis_Z

        # Mempublikasikan ke cmd_vel
        self.publisher_axis.publish(twist)

        # Logging informasi
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
