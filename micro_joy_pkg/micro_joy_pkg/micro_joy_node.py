import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray

import serial
import time

# Buka port serial dengan Arduino
arduino = serial.Serial(port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',   
                        baudrate=115200, 
                        timeout=.1)

time.sleep(2)  # Tunggu agar Arduino siap

class Joy(Node):
    def __init__(self):
        super().__init__('micro_joy_node')
        self.publisher_=self.create_publisher(Float32MultiArray,'/topic',10)
        self.timer = self.create_timer(0, self.publish_array)

    def read(self):
        data = arduino.readline()  # Baca data dari Arduino
        # print(f"Raw data: {data}")  # Cetak data mentah untuk debugging
        try:
            decoded_data = data.decode('utf-8').strip()  # Decode dan hilangkan newline/whitespace
            return decoded_data  # Kembalikan string data mentah
        except (UnicodeDecodeError, ValueError) as e:
            print(f"Error processing data: {e}")
            return None  # Kembalikan None jika ada kesalahan
        
    def publish_array(self):
        msg = Float32MultiArray()
        response = self.read()
        
        if response is not None:
            values = response.split(",")
            # print("ini nilai X",float(values[0]))
            if len(values) == 11:  # Sesuaikan dengan jumlah elemen yang diharapkan
                try:
                    msg.data = [float(values[i]) for i in range(11)]  # Konversi menjadi float
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
                except ValueError:
                    self.get_logger().warn('Invalid float values in response')
            else:
                self.get_logger().warn(f"Expected 11 values, but got: {len(values)}")
        else:
            self.get_logger().warn('No data received or error in reading data')


def main(args=None):
    rclpy.init(args=args)
    node = Joy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()