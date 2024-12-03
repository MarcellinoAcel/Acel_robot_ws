import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from sklearn.linear_model import LinearRegression

class PIDRegressionNode(Node):
    def __init__(self):
        super().__init__('pid_regression_node')
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'data_for_regres',
            self.data_callback,
            10
        )
        self.pid_publisher = self.create_publisher(
            Float32MultiArray,
            'pid_parameters',
            10
        )
        self.data_buffer = []  # Sliding window for storing recent data
        self.window_size = 100  # Max number of samples to store
        self.get_logger().info("PID Regression Node Started!")

    def data_callback(self, msg):
        # Parse incoming data
        if len(msg.data) != 4:
            self.get_logger().error("Expected 4 values in the message: [e, integral, derivative, control_signal]")
            return
        
        e, integral, derivative, u = msg.data
        self.data_buffer.append([e, integral, derivative, u])
        
        # Maintain sliding window
        if len(self.data_buffer) > self.window_size:
            self.data_buffer.pop(0)
        
        # Perform regression if sufficient data
        if len(self.data_buffer) >= 10:  # Minimum samples required
            self.perform_regression()

    def perform_regression(self):
        # Prepare data
        data = np.array(self.data_buffer)
        X = data[:, :3]  # [e, integral, derivative]
        y = data[:, 3]   # control_signal (u)

        # Linear regression
        model = LinearRegression()
        model.fit(X, y)

        # Extract PID parameters
        K_p, K_i, K_d = model.coef_
        self.get_logger().info(f"Estimated PID parameters - Kp: {K_p}, Ki: {K_i}, Kd: {K_d}")

        # Publish PID parameters
        pid_msg = Float32MultiArray()
        pid_msg.data = [K_p, K_i, K_d]
        self.pid_publisher.publish(pid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDRegressionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting_down_linear regression")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
