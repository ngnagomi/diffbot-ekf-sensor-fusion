"""
Drive the robot in a repeatable pattern.
Open-loop: every run sends exactly the same commands, timed with simulation time.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PathDriver(Node):

    def __init__(self):
        super().__init__('path_driver')

        # params can be set in command line
        self.declare_parameter('pattern', 'square')  # square, star, waypoint, circle, fgure8
        self.declare_parameter('speed', 0.2)         # forward speed, m/s
        self.declare_parameter('turn_rate', 0.5)     # turning speed, rad/s
        self.declare_parameter('laps', 1)            # how many times to repeat the pattern
        self.declare_parameter('side', 2.0)          # square: side length, m
        self.declare_parameter('size', 1.0)          # star: radius, m
        self.declare_parameter('points', '')         # waypoints: "x1,y1;x2,y2;..." in m
        self.declare_parameter('turn_scale', 1.0)    # >1 lengthens turns to make up for wheel slip

        # runs as a list of (duration_s, linear_x, angular_z)
        self.segments = self.build_segments()
        total = sum(seg[0] for seg in self.segments)
        self.get_logger().info(
            f'Pattern "{self.param("pattern")}": {len(self.segments)} segments, '
            f'{total:.1f} s of sim time')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.index = 0               # which segment we're on
        self.segment_start = None    # sim time when the current segment began
        self.done = False
        self.timer = self.create_timer(0.02, self.tick)   # run tick() 50 times per second

    def param(self, name):
        return self.get_parameter(name).value

    # building list of segments

    def build_segments(self):
        pattern = self.param('pattern')
        speed = self.param('speed')
        turn_rate = self.param('turn_rate')
        laps = self.param('laps')

        segments = [(1.0, 0.0, 0.0)]   # wait 1 s at the start so everything settles

        if pattern == 'square':
            s = self.param('side')
            corners = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
            segments += self.polyline_segments(corners, speed, turn_rate, laps)

        elif pattern == 'star':
            points = self.star_points(self.param('size'))
            segments += self.polyline_segments(points, speed, turn_rate, laps)

        elif pattern == 'waypoints':
            points = self.parse_points(self.param('points'))
            segments += self.polyline_segments(points, speed, turn_rate, laps)

        elif pattern == 'circle':
            loop_time = 2 * math.pi / turn_rate          # time for one full 360-degree loop
            segments += [(loop_time * laps, speed, turn_rate)]

        elif pattern == 'figure8':
            loop_time = 2 * math.pi / turn_rate
            for _ in range(laps):
                segments += [(loop_time, speed, turn_rate),     # loop to the left
                             (loop_time, speed, -turn_rate)]    # loop to the right

        else:
            raise ValueError(f'Unknown pattern "{pattern}" '
                             '(use square, star, waypoints, circle or figure8)')
        return segments

    def polyline_segments(self, points, speed, turn_rate, laps):
        # Turn a list of (x, y) points into turn-then-drive segments
        pause = (0.5, 0.0, 0.0)
        turn_scale = self.param('turn_scale')
        segments = []
        x, y, heading = 0.0, 0.0, 0.0          # robot starts at the origin, facing +x
        for _ in range(laps):
            for px, py in points:
                dx, dy = px - x, py - y
                dist = math.hypot(dx, dy)
                if dist < 1e-6:
                    continue                   # already at this point
                target = math.atan2(dy, dx)    # direction to the next point
                # Turn needed, wrapped to [-pi, pi] so we always turn the short way
                turn = math.atan2(math.sin(target - heading), math.cos(target - heading))
                if abs(turn) > 1e-3:
                    turn_time = abs(turn) / turn_rate * turn_scale
                    segments += [(turn_time, 0.0, math.copysign(turn_rate, turn)), pause]
                segments += [(dist / speed, speed, 0.0), pause]
                x, y, heading = px, py, target
        return segments

    def star_points(self, size):
        # Five-pointed star: visit every second corner of a pentagon.
        corners = [(size * math.cos(math.radians(90 + 72 * k)),
                    size * math.sin(math.radians(90 + 72 * k))) for k in range(5)]
        order = [0, 2, 4, 1, 3, 0]
        x0, y0 = corners[0]
        return [(corners[i][0] - x0, corners[i][1] - y0) for i in order]  # start at (0, 0)

    def parse_points(self, text):
        #'0,0;0,2;1.2,0' -> [(0.0, 0.0), (0.0, 2.0), (1.2, 0.0)]
        if not text:
            raise ValueError('pattern:=waypoints needs -p "points:=x1,y1;x2,y2;..."')
        return [tuple(float(v) for v in pair.split(',')) for pair in text.split(';')]

    # Playing the segments back

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return     

        if self.segment_start is None:
            self.segment_start = now

        duration, v, w = self.segments[self.index]
        elapsed = (now - self.segment_start).nanoseconds / 1e9

        if elapsed >= duration:                     
            self.index += 1
            if self.index >= len(self.segments):    
                self.publisher.publish(Twist())     
                self.get_logger().info('Pattern complete, robot stopped.')
                self.timer.cancel()
                self.done = True
                return
            self.segment_start = now
            duration, v, w = self.segments[self.index]

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PathDriver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.5)       
    except KeyboardInterrupt:
        node.publisher.publish(Twist())      
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()