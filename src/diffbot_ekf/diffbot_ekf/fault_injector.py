import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

FAULTS = ('none', 'wheel_scale', 'wheel_slip', 'imu_spikes', 'imu_dropout')


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        # Fault type & strength
        self.declare_parameter('fault', 'none')
        self.declare_parameter('wheel_scale', 1.05)    # wheel_scale: wheels report 5% too much motion
        self.declare_parameter('slip_speed', 0.15)     # wheel_slip: fake extra forward speed, m/s
        self.declare_parameter('spike_size', 3.0)      # imu_spikes: size of one spike, rad/s
        self.declare_parameter('spike_chance', 0.02)   # imu_spikes: chance any one reading is a spike
        self.declare_parameter('window_start', 5.0)    # timed faults: first window, s after the robot first moves
        self.declare_parameter('window_period', 10.0)  # timed faults: a window starts this often, s
        self.declare_parameter('window_length', 3.0)   # timed faults: each window lasts this long, s
        self.declare_parameter('seed', 42)             # same seed -> same "random" spikes every run
        self.declare_parameter('wheel_vyaw_std', 0.2)

        self.fault = self.get_parameter('fault').value
        if self.fault not in FAULTS:
            raise ValueError('Unknown fault "' + self.fault + '". Use one of: ' + ', '.join(FAULTS))
        self.rng = random.Random(self.get_parameter('seed').value)

        self.move_start = None        
    
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_wheel_time = None

        self.odom_pub = self.create_publisher(Odometry, '/ekf_input/wheel/odometry', 10)
        self.imu_pub = self.create_publisher(Imu, '/ekf_input/imu/data', 10)
        self.create_subscription(Odometry, '/wheel/odometry', self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.get_logger().info('Fault injector running, fault: ' + self.fault)

    def param(self, name):
        return self.get_parameter(name).value

    # When are timed faults active?

    def on_cmd(self, msg):
        moving = abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6
        if moving and self.move_start is None:
            self.move_start = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info('Robot started moving; fault timing starts now.')

    def in_window(self, t):
        """True while a timed fault (slip burst or dropout) is active at sim time t."""
        if self.move_start is None:
            return False
        since = t - self.move_start - self.param('window_start')
        if since < 0:
            return False
        return (since % self.param('window_period')) < self.param('window_length')

    # Wheel odometry

    def on_wheel(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        if self.fault == 'wheel_scale':
            s = self.param('wheel_scale')    
            vx = vx * s
            wz = wz * s
        elif self.fault == 'wheel_slip' and self.in_window(t):
            vx = vx + self.param('slip_speed')  

        if self.last_wheel_time is not None:
            dt = t - self.last_wheel_time
            if dt > 0:
                mid_yaw = self.yaw + 0.5 * wz * dt
                self.x += vx * math.cos(mid_yaw) * dt
                self.y += vx * math.sin(mid_yaw) * dt
                self.yaw += wz * dt
        self.last_wheel_time = t

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = math.sin(self.yaw / 2)   
        out.pose.pose.orientation.w = math.cos(self.yaw / 2)
        out.pose.covariance = msg.pose.covariance
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = wz

        twist_cov = list(msg.twist.covariance) # 6x6 matrix, flattened to 36 numbers
        std = self.param('wheel_vyaw_std')
        if std > 0:
            twist_cov[35] = std ** 2 # position 35 = the vyaw variance
        out.twist.covariance = twist_cov

        self.odom_pub.publish(out)

    # IMU

    def on_imu(self, msg):
        t = stamp_to_sec(msg.header.stamp)

        if self.fault == 'imu_dropout' and self.in_window(t):
            return                           

        if self.fault == 'imu_spikes' and self.rng.random() < self.param('spike_chance'):
            sign = self.rng.choice((-1.0, 1.0))
            msg.angular_velocity.z += sign * self.param('spike_size')

        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()