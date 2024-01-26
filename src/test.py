import rclpy
from std_msgs.msg import String

def publish_message():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('orga_pub')

    # Create a publisher for the specified topic
    topic_name = 'orga_topic'
    publisher = node.create_publisher(String, topic_name, 10)

    # Publish a string message
    message = String()
    message.data = 'random'
    publisher.publish(message)
    print(f"Published message: {message.data}")

    # Keep the node running
    rclpy.spin(node)

    # Cleanup (this will not be executed until the node is manually stopped)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_message()
