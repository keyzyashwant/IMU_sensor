import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.pub = self.create_publisher(Imu, '/imu/raw', 10)

        # ── Find your port first: run "ls /dev/ttyACM*" in terminal ──
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.01, self.read_serial)   # 100 Hz
        self.get_logger().info("Serial IMU node started on /dev/ttyACM0")

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split(',')

            if len(parts) != 4:   # expect: timestamp,ax,ay,az
                return

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            # Parts: [timestamp_ms, ax, ay, az]
            msg.linear_acceleration.x = float(parts[1])
            msg.linear_acceleration.y = float(parts[2])
            msg.linear_acceleration.z = float(parts[3])

            # Required fields — set to unknown (covariance = -1)
            msg.orientation_covariance[0]         = -1.0
            msg.angular_velocity_covariance[0]    = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.pub.publish(msg)

        except (ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Bad serial line: {e}')

def main():
    rclpy.init()
    rclpy.spin(ImuSerialNode())

if __name__ == '__main__':
    main()