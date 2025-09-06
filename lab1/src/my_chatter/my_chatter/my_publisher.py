# These lines allow us to import rclpy so we can use Python and its Node class
import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

# This line imports the built-in string message type that our node will use to structure its data to pass on our topic
from std_msgs.msg import String

# We're creating a class called Talker, which is a subclass of Node
class MyPublisher(Node):

    # Here, we define the constructor
    def __init__(self):
        # We call the Node class's constructor and call it "my_publisher"
        super().__init__('my_publisher')

         # Here, we set that the node publishes message of type String (where did this type come from?), over a topic called "chatter_talk", and with queue size 10. The queue size limits the amount of queued messages if a subscriber doesn't receive them quickly enough.
        self.publisher_ = self.create_publisher(TimestampString, 'user_messages', 10)

    def send_user_input(self):
        user_text = input("Please enter a line of text and press <Enter>: ")
        timestamp = float(self.get_clock().now().nanoseconds)

        msg = TimestampString()
        msg.text = user_text
        msg.timestamp = timestamp

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published message: '{msg.text}' at {msg.timestamp}")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    my_publisher = MyPublisher()

    try:
        while rclpy.ok():
            my_publisher.send_user_input()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        my_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

