#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self, turtle_name):
        super().__init__("turtle_controller")
        topic_name = f"/{turtle_name}/cmd_vel"
        self.publisher_ = self.create_publisher(Twist, topic_name, 10)
        self.get_logger().info(f"Controlling {turtle_name} on topic {topic_name}")

    def run(self):
        while rclpy.ok():
            cmd = input("Enter command (w/a/s/d to move, q to quit): ").strip()
            msg = Twist()

            if cmd == "w":
                msg.linear.x = 2.0
            elif cmd == "s":
                msg.linear.x = -2.0
            elif cmd == "a":
                msg.angular.z = 2.0
            elif cmd == "d":
                msg.angular.z = -2.0
            elif cmd == "q":
                self.get_logger().info("Exiting controller.")
                break
            else:
                self.get_logger().info("Invalid input.")

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run lab2_turtlesim turtle_controller <turtle_name>")
        return

    turtle_name = sys.argv[1]
    node = TurtleController(turtle_name)
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

