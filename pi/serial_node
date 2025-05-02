import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class AngleSerialNode(Node):
    def __init__(self):
        super().__init__('angle_serial_node')

        # Serial connection (adjust port name if needed)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Subscription to angle array topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/angles',  # Expecting a 12-element array
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        angles = msg.data

        if len(angles) != 12:
            self.get_logger().warn(f"Expected 12 angles but got {len(angles>            return

        angle_str = ','.join(f"{a:.2f}" for a in angles)  # Format: 45.00,3>        self.get_logger().info(f"Sending angles: {angle_str}")              
        try:
            self.serial_port.write(f"ANG:{angle_str}\n".encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AngleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
