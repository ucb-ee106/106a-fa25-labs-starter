#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class TurtlePatrolServer(Node):
    def __init__(self):
        super().__init__('turtle_patrol_server')

        # Store publishers, clients, and current commands per turtle
        self._cmd_publishers = {}
        self._teleport_clients = {}
        self._current_cmds = {}

        # Create the patrol service
        self._srv = self.create_service(Patrol, '/turtle_patrol', self.patrol_callback)

        # Timer: publish all stored commands at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmds)

        self.get_logger().info('TurtlePatrolServer ready (continuous publish mode).')

    # -------------------------------------------------------
    # Publish stored velocity commands for all turtles
    # -------------------------------------------------------
    def _publish_current_cmds(self):
        for turtle_name, cmd in self._current_cmds.items():
            pub = self._cmd_publishers.get(turtle_name)
            if pub is not None:
                pub.publish(cmd)

    # -------------------------------------------------------
    # Get/create publisher for a turtle's /cmd_vel
    # -------------------------------------------------------
    def _get_cmd_publisher(self, turtle_name: str):
        if turtle_name not in self._cmd_publishers:
            topic = f'/{turtle_name}/cmd_vel'
            self._cmd_publishers[turtle_name] = self.create_publisher(Twist, topic, 10)
            self.get_logger().info(f"Created publisher for {topic}")
        return self._cmd_publishers[turtle_name]

    # -------------------------------------------------------
    # Get/create teleport client for a turtle
    # -------------------------------------------------------
    def _get_teleport_client(self, turtle_name: str):
        if turtle_name not in self._teleport_clients:
            service_name = f'/{turtle_name}/teleport_absolute'
            client = self.create_client(TeleportAbsolute, service_name)
            self._teleport_clients[turtle_name] = client
            self.get_logger().info(f"Created client for {service_name}")
        return self._teleport_clients[turtle_name]

    # -------------------------------------------------------
    # Service callback
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        turtle_name = request.turtle_name

        self.get_logger().info(
            f"Patrol request for {turtle_name}: "
            f"vel={request.vel:.2f}, omega={request.omega:.2f}, "
            f"x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}"
        )

        # Ensure publisher and teleport client exist
        cmd_pub = self._get_cmd_publisher(turtle_name)
        teleport_client = self._get_teleport_client(turtle_name)

        # Teleport the turtle
        if not teleport_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Teleport service for {turtle_name} not available.")
        else:
            teleport_req = TeleportAbsolute.Request()
            teleport_req.x = request.x
            teleport_req.y = request.y
            teleport_req.theta = request.theta
            teleport_client.call_async(teleport_req)
            self.get_logger().info(
                f"Teleporting {turtle_name} to ({request.x:.2f}, {request.y:.2f}, {request.theta:.2f})"
            )

        # Store velocity command for continuous publishing
        cmd = Twist()
        cmd.linear.x = request.vel
        cmd.angular.z = request.omega
        self._current_cmds[turtle_name] = cmd

        # Respond with the current Twist
        response.cmd = cmd
        self.get_logger().info(
            f"Streaming cmd_vel for {turtle_name}: lin.x={cmd.linear.x:.2f}, ang.z={cmd.angular.z:.2f}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
