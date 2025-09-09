#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol


class PatrolClient(Node):
    def __init__(self, turtle_name, x, y, theta, vel, omega):
        super().__init__('patrol_client')

        # Create service client
        self.cli = self.create_client(Patrol, '/turtle_patrol')

        # Wait until service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Prepare request
        self.req = Patrol.Request()
        self.req.turtle_name = turtle_name
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.theta = float(theta)
        self.req.vel = float(vel)
        self.req.omega = float(omega)

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 7:
        print("Usage: ros2 run turtle_patrol patrol_client <turtle_name> <x> <y> <theta> <velocity> <omega>")
        return

    turtle_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    theta = sys.argv[4]
    vel = sys.argv[5]
    omega = sys.argv[6]

    client = PatrolClient(turtle_name, x, y, theta, vel, omega)
    response = client.send_request()

    if response:
        print(f"Service response:\n  linear: {response.cmd.linear}\n  angular: {response.cmd.angular}")
    else:
        print("Service call failed")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
