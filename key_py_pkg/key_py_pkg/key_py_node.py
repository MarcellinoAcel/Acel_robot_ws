import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class KeyboardRos(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()

        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Timer for publishing messages at a fixed rate
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_message)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char == 'w':
                self.msg.linear.x = 1.0
                self.msg.linear.y = 0.0
                self.msg.angular.z = 0.0
            elif hasattr(key, 'char') and key.char == 's':
                self.msg.linear.x = -1.0
                self.msg.linear.y = 0.0
                self.msg.angular.z = 0.0
            elif hasattr(key, 'char') and key.char == 'a':
                self.msg.linear.x = 0.0
                self.msg.linear.y = -1.0
                self.msg.angular.z = 0.0
            elif hasattr(key, 'char') and key.char == 'd':
                self.msg.linear.x = 0.0
                self.msg.linear.y = 1.0
                self.msg.angular.z = 0.0
            elif hasattr(key, 'char') and key.char == 'q':
                self.msg.angular.z = -1.0
                self.msg.linear.x = 0.0
                self.msg.linear.y = 0.0
            elif hasattr(key, 'char') and key.char == 'e':
                self.msg.angular.z = 1.0
                self.msg.linear.x = 0.0
                self.msg.linear.y = 0.0
        except AttributeError:
            pass
    def on_release(self, key):
        try:
            # Reset velocity when keys are released
            if hasattr(key, 'char') and key.char in ['w', 's','a', 'd', 'q', 'e']:
                self.msg.angular.z = 0.0
                self.msg.linear.x = 0.0
                self.msg.linear.y = 0.0
        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            return False

    def publish_message(self):
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_ros = KeyboardRos()
    rclpy.spin(keyboard_ros)
    keyboard_ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
