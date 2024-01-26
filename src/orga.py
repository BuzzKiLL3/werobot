import rclpy
from std_msgs.msg import String
import subprocess

def start_program(program_path):
    print(f"Starting move: {program_path}")
    subprocess.Popen(["python", program_path])

def message_callback(msg):
    message = msg.data
    print(f"Received message: {message}")

    if message == "start_program1":
        start_program("path/to/program1.py")
    elif message == "start_program2":
        start_program("path/to/program2.py")
    elif message == "stop_program":
        print("Stopping the program...")
        # Perform the task for stopping the program
    else:
        print("Unknown command. Ignoring...")

def main():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('subscriber_node')

    # Create a subscriber for the specified topic
    topic_name = 'your_topic'
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
