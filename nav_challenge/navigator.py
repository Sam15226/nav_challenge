import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

WAYPOINTS = [(4.0, 5.0), (12.0, 3.0), (15.0, -3.0)]

WALLS = [
    (4.5, 5.5, -3.0, 3.0),
    (9.5, 10.5, -6.0, 0.0),
]

LOOKAHEAD = 1.5  # meters -- how far ahead we check for obstacles

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.pub = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/model/vehicle/odometry', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wp_index = 0
        self.avoiding = False
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def point_in_wall(self, px, py):
        for (xmin, xmax, ymin, ymax) in WALLS:
            if xmin <= px <= xmax and ymin <= py <= ymax:
                return True
        return False

    def path_blocked(self):
        # Check a point further out in front of the vehicle
        check_x = self.x + LOOKAHEAD * math.cos(self.yaw)
        check_y = self.y + LOOKAHEAD * math.sin(self.yaw)
        return self.point_in_wall(check_x, check_y)

    def control_loop(self):
        if self.wp_index >= len(WAYPOINTS):
            self.pub.publish(Twist())
            return

        gx, gy = WAYPOINTS[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.4:
            self.get_logger().info(f'Reached waypoint {self.wp_index}: ({gx},{gy})')
            self.wp_index += 1
            self.avoiding = False
            return

        cmd = Twist()

        if self.path_blocked():
            self.avoiding = True
            self.get_logger().info('BLOCKED - turning to avoid obstacle')
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0  # turn in place until clear
        elif self.avoiding:
            # just cleared an obstacle -- creep forward a bit before re-aiming at goal
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.avoiding = False
        else:
            target_yaw = math.atan2(dy, dx)
            yaw_error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            if abs(yaw_error) > 0.15:
                cmd.linear.x = 0.1
                cmd.angular.z = 1.2 * yaw_error
            else:
                cmd.linear.x = 1.0
                cmd.angular.z = 1.2 * yaw_error

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
