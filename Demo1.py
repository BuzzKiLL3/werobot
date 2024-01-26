from pyniryo2 import *
import rclpy
from std_msgs.msg import String

# - Constants
workspace_name = "workspace_fixe_V2"  # Robot's Workspace Name
robot_ip_address = "10.89.5.15"

# The pose from where the image processing happens
observation_pose = PoseObject(
    x=0.024, y=0.2, z=0.25,
    roll=-2.939, pitch=1.359, yaw=-1.352,
)

#Intermediate place
intermediate_place = PoseObject(
    x=0.14, y=0, z=0.2,
    roll=1.49, pitch=1.54, yaw=1.57
)

#center mobile robot position
mobile_robot_point = [-0.08, -1.44,-0.29, -0.18, 0.15, 0.02]

# Place pose square
place_pose_square_red = PoseObject(
    x=0.198, y=0.033, z=-0.0992,
    roll=-2.853, pitch=1.473, yaw=-2.799
)

# Place pose circle
place_pose_circle_red=place_pose_square_red.copy_with_offsets(z_offset=0.02)

# Place pose square bleu
place_pose_square_bleu=place_pose_square_red.copy_with_offsets(x_offset=0.05)

# Place pose square bleu
place_pose_circle_bleu=place_pose_square_bleu.copy_with_offsets(z_offset=0.02)
# Place pose square bleu
place_pose_square_green=place_pose_square_red.copy_with_offsets(y_offset=0.05)
# Place pose square bleu
place_pose_circle_green=place_pose_square_green.copy_with_offsets(z_offset=0.02)
# - Initialization

# Connect to robot
robot = NiryoRobot(robot_ip_address)
# Calibrate robot if robot needs calibration
robot.arm.calibrate_auto()
# Updating tool
robot.tool.update_tool()

###############################3


max_failure_count = 3


#Functions


def get_object(object_shape, object_color,place):
    try_without_success = 0
    while try_without_success < max_failure_count:
    # Moving to observation pose
        robot.arm.move_pose(observation_pose)
    # Trying to get object via Vision Pick
        obj_found, shape, color = robot.vision.vision_pick(workspace_name, shape = object_shape, color = object_color)
        if not obj_found:
            try_without_success += 1
            robot.wait(0.1)
            continue  

        robot.arm.move_pose(intermediate_place)
        robot.arm.move_pose(place)
        robot.tool.grasp_with_tool()
        robot.tool.open_gripper(speed=500)
        robot.arm.move_pose(intermediate_place)

get_object(ObjectShape.SQUARE,ObjectColor.RED,place_pose_square_red)
get_object(ObjectShape.SQUARE, ObjectColor.GREEN,place_pose_square_green)
get_object(ObjectShape.SQUARE, ObjectColor.BLUE,place_pose_square_bleu)
get_object(ObjectShape.CIRCLE,ObjectColor.BLUE,place_pose_circle_bleu)
get_object(ObjectShape.CIRCLE, ObjectColor.GREEN,place_pose_circle_green)
get_object(ObjectShape.CIRCLE, ObjectColor.RED,place_pose_circle_red)

robot.arm.go_to_sleep()

robot.end()


def publish_message():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('orga_pub')

    # Create a publisher for the specified topic
    topic_name = 'orga_topic'
    publisher = node.create_publisher(String, topic_name, 10)

    # Publish a string message
    message = String()
    message.data = 'arm_done'
    publisher.publish(message)
    print(f"Published message: {message.data}")

    # Keep the node running
    rclpy.spin(node)

    # Cleanup (this will not be executed until the node is manually stopped)
    node.destroy_node()
    rclpy.shutdown()

#after finishing your condition add this command
# publish_message()