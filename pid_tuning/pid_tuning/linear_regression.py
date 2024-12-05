import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sklearn.linear_model import LinearRegression
import numpy as np

class PIDRegressionNode(Node):
    def __init__(self):
        super().__init__('pid_regression_node')

        # Data buffers for regression
        self.error_data = []
        self.derivative_data = []
        self.integral_data = []
        self.control_effort_data = []

        # Subscriptions
        self.error_sub = self.create_subscription(
            Float32, 'error2tune', self.error_callback, 10)
        self.control_effort_sub = self.create_subscription(
            Float32, 'controlled2tune', self.control_effort_callback, 10)

        self.pid_params_pub = self.create_publisher(
            Float32MultiArray, 'pid_parameters', 10)
        
        # Timer for performing regression
        self.timer = self.create_timer(5.0, self.perform_regression)

        # PID parameters (defaults)
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0

    def error_callback(self, msg):
        # Update error buffer
        error = msg.data
        if len(self.error_data) > 0:
            dt = 0.1  # Assume time step (or measure from timestamps)
            derivative = (error - self.error_data[-1]) / dt
        else:
            derivative = 0.0

        integral = sum(self.error_data) * 0.1  # Approximate integral
        self.error_data.append(error)
        self.derivative_data.append(derivative)
        self.integral_data.append(integral)

    def control_effort_callback(self, msg):
        # Update control effort buffer
        self.control_effort_data.append(msg.data)

    def perform_regression(self):
        # Check if enough data is available
        if len(self.error_data) < 10:
            self.get_logger().info("Not enough data for regression.")
            return

        # Prepare data for regression
        X = np.column_stack([
            self.error_data,
            self.derivative_data,
            self.integral_data
        ])
        y = np.array(self.control_effort_data)

        # Perform linear regression
        model = LinearRegression()
        model.fit(X, y)

        # Extract coefficients as PID parameters
        self.Kp, self.Kd, self.Ki = model.coef_
        self.get_logger().info(f"Updated PID Gains: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

        # Reset buffers
        self.error_data = []
        self.derivative_data = []
        self.integral_data = []
        self.control_effort_data = []

        # Update PID parameters dynamically
        self.publish_pid_parameters()

    
    def publish_pid_parameters(self):
        # Publish the updated PID parameters as a Float32MultiArray
        msg = Float32MultiArray()
        msg.data = [self.Kp, self.Ki, self.Kd]
        self.pid_params_pub.publish(msg)
        self.get_logger().info("Published updated PID parameters.")

def main(args=None):
    rclpy.init(args=args)
    node = PIDRegressionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
