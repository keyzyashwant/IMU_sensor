import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')

        self.sub = self.create_subscription(Imu, '/imu/raw', self.imu_cb, 10)
        self.kf_pub  = self.create_publisher(Path, '/kf_path',  10)
        self.raw_pub = self.create_publisher(Path, '/raw_path', 10)

        self.dt = 0.01

        # ── Kalman filter state (2D: x and y independent) ──────────
        # State: [position, velocity] per axis
        self.F = np.array([[1, self.dt],
                           [0, 1      ]])
        self.B = np.array([[0.5 * self.dt**2],
                           [self.dt         ]])
        self.H = np.array([[1, 0]])
        self.Q = np.array([[0.001, 0   ],
                           [0,     0.01]])
        self.R = np.array([[0.5]])
        self.I = np.eye(2)

        # Separate state + covariance for x and y axes
        self.x_x = np.zeros((2, 1))   # [pos_x, vel_x]
        self.P_x = np.eye(2)

        self.x_y = np.zeros((2, 1))   # [pos_y, vel_y]
        self.P_y = np.eye(2)

        # Dead reckoning state
        self.dr_vx, self.dr_vy = 0.0, 0.0
        self.dr_px, self.dr_py = 0.0, 0.0

        # Path messages (accumulate poses over time)
        self.kf_path  = Path()
        self.raw_path = Path()
        self.kf_path.header.frame_id  = 'map'
        self.raw_path.header.frame_id = 'map'

        self.get_logger().info("Kalman node started")

    def kalman_update(self, x, P, accel):
        """Run one step of KF for a single axis."""
        u = np.array([[accel]])

        # PREDICT
        x_pred = self.F @ x + self.B @ u
        P_pred = self.F @ P @ self.F.T + self.Q

        # UPDATE (use predicted position as mock measurement)
        # In real life: replace x_pred[0,0] with GPS fix
        z = np.array([[x_pred[0, 0] + np.random.normal(0, 0.1)]])
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        x = x_pred + K @ (z - self.H @ x_pred)
        P = (self.I - K @ self.H) @ P_pred

        return x, P

    def imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        now = msg.header.stamp

        # ── Kalman filter update ──────────────────────────────────
        self.x_x, self.P_x = self.kalman_update(self.x_x, self.P_x, ax)
        self.x_y, self.P_y = self.kalman_update(self.x_y, self.P_y, ay)

        kf_px = self.x_x[0, 0]
        kf_py = self.x_y[0, 0]

        # ── Dead reckoning update ─────────────────────────────────
        self.dr_vx += ax * self.dt
        self.dr_vy += ay * self.dt
        self.dr_px += self.dr_vx * self.dt
        self.dr_py += self.dr_vy * self.dt

        # ── Build KF pose and append to path ─────────────────────
        kf_pose = PoseStamped()
        kf_pose.header.stamp    = now
        kf_pose.header.frame_id = 'map'
        kf_pose.pose.position.x = kf_px
        kf_pose.pose.position.y = kf_py
        kf_pose.pose.orientation.w = 1.0
        self.kf_path.poses.append(kf_pose)

        # ── Build DR pose and append to path ─────────────────────
        dr_pose = PoseStamped()
        dr_pose.header.stamp    = now
        dr_pose.header.frame_id = 'map'
        dr_pose.pose.position.x = self.dr_px
        dr_pose.pose.position.y = self.dr_py
        dr_pose.pose.orientation.w = 1.0
        self.raw_path.poses.append(dr_pose)

        # ── Publish both paths ────────────────────────────────────
        self.kf_path.header.stamp  = now
        self.raw_path.header.stamp = now
        self.kf_pub.publish(self.kf_path)
        self.raw_pub.publish(self.raw_path)

def main():
    rclpy.init()
    rclpy.spin(KalmanNode())

if __name__ == '__main__':
    main()