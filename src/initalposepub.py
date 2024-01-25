import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # Create and publish a single message
        self.publish_initial_pose()

    def publish_initial_pose(self):
        message = PoseWithCovarianceStamped()
        # Set the necessary fields in the message
        # For example, setting the position x, y, z
        message.pose.pose.position.x = 1.0
        message.pose.pose.position.y = 2.0
        message.pose.pose.position.z = 0.0

        # Set the orientation (quaternion) if needed
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = 0.0
        message.pose.pose.orientation.w = 1.0

        # Publish the message
        self.publisher.publish(message)
        self.get_logger().info('Published initial pose')

def main(args=None):
    rclpy.init(args=args)

    node = InitialPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
