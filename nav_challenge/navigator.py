import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random

def generate_waypoints(count=3):
    regions = [
        (1.5, 3.5, -2.0, 2.0),
        (6.5, 8.5, -2.0, 2.0),
        (11.0, 14.0, 1.0, 5.0),
    ]

    waypoints = []

    for i in range(count):
        xmin, xmax, ymin, ymax = regions[i % len(regions)]
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        waypoints.append((round(x, 1), round(y, 1)))

    return waypoints

WAYPOINTS = generate_waypoints()


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.pub = self.create_publisher(
            Twist,
            '/model/vehicle/cmd_vel',
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            '/model/vehicle/odometry',
            self.odom_cb,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wp_index = 0
        self.route = []
        self.route_index = 0

        self.build_route()

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def build_route(self):

        self.route = []

        for i, waypoint in enumerate(WAYPOINTS):

            self.route.append(waypoint)

            if i < len(WAYPOINTS) - 1:

                current = waypoint
                next_wp = WAYPOINTS[i + 1]

                if current[0] < 4.5 and next_wp[0] > 5.5:

                    self.route.append((3.5, 4.5))
                    self.route.append((6.5, 4.5))

                elif current[0] < 9.5 and next_wp[0] > 10.5:

                    self.route.append((8.5, 1.5))
                    self.route.append((11.5, 1.5))

        self.get_logger().info(
            f'Random waypoints: {WAYPOINTS}'
        )

        self.get_logger().info(
            f'Navigation route: {self.route}'
        )

    def drive_to(self, tx, ty):

        cmd = Twist()

        dx = tx - self.x
        dy = ty - self.y

        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)

        yaw_error = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw)
        )

        if distance < 0.5:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif abs(yaw_error) > 0.15:

            cmd.linear.x = 0.1
            cmd.angular.z = 1.5 * yaw_error

        else:

            cmd.linear.x = 0.8
            cmd.angular.z = 1.0 * yaw_error

        return cmd

    def control_loop(self):

        if self.route_index >= len(self.route):

            self.pub.publish(Twist())

            return

        tx, ty = self.route[self.route_index]

        distance = math.hypot(
            tx - self.x,
            ty - self.y
        )

        if distance < 0.5:

            self.get_logger().info(
                f'Reached route point {self.route_index}: ({tx}, {ty})'
            )

            self.route_index += 1

            self.pub.publish(Twist())

            return

        cmd = self.drive_to(tx, ty)

        self.pub.publish(cmd)


def main():

    rclpy.init()

    node = Navigator()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
