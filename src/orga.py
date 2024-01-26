import rclpy
from std_msgs.msg import String
import subprocess

def start_program(program_path):
    print(f"Starting move: {program_path}")
    subprocess.Popen(["python", program_path])

def message_callback(msg):
    message = msg.data
    print(f"Received message: {message}")

    if message == "init":
        start_program("initialposepub.py")
    elif message == "random":
        start_program("move.py")
    elif message == "stop_program":
        print("Stopping the program...")
        # Perform the task for stopping the program
    else:
        print("Unknown command. Ignoring...")

def main():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('orga_sub')

    # Create a subscriber for the specified topic
    topic_name = 'orga_topic'
    subscriber = node.create_subscription(String, topic_name, message_callback, 10)

    # Print the topic being subscribed to
    print(f"Subscribed to topic: {topic_name}")

    try:
        # Spin the node to keep it running
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("Shutting down the node...")

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
