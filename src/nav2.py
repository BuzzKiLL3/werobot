#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def main():
    rclpy.init()

    # Create the ActionClient for NavigateToPose
    navigate_client = ActionClient(rclpy.create_node('navigate_to_pose_client'), NavigateToPose, '/navigate_to_pose')

    # Create a publisher for setting initial pose to AMCL
    amcl_pose_publisher = rclpy.create_node('amcl_pose_publisher')
    amcl_pose_pub = amcl_pose_publisher.create_publisher(PoseWithCovarianceStamped, '/initialpose1', 10)

    # Wait for the action server to come up
    while not navigate_client.wait_for_server(timeout_sec=5.0):
        print("Waiting for the navigate_to_pose action server to come up")

    # Set the initial pose for AMCL
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = 'map'  # Assuming AMCL uses 'map' frame for localization
    initial_pose_msg.header.stamp = rclpy.Time.now().to_msg()
    initial_pose_msg.pose.pose.position.x = 0.0  # Set the initial x position
    initial_pose_msg.pose.pose.position.y = 0.0  # Set the initial y position
    initial_pose_msg.pose.pose.orientation.z = 0.0  # Set the initial orientation (assuming no rotation)
    initial_pose_msg.pose.pose.orientation.w = 1.0  # Quaternion's w component should be 1 for no rotation

    amcl_pose_pub.publish(initial_pose_msg)

    # Create a goal for NavigateToPose
    navigate_goal = NavigateToPose.Goal()
    navigate_goal.pose.header.frame_id = 'map'  # Assuming 'map' frame is used by AMCL
    navigate_goal.pose.header.stamp = rclpy.Time.now().to_msg()
    navigate_goal.pose.pose.position.x = 1.0
    navigate_goal.pose.pose.orientation.w = 1.0

    print("Sending NavigateToPose goal")
    navigate_client.send_goal(navigate_goal)

    # Wait for the result
    navigate_client.wait_for_result()

    # Check if the goal was successful
    if navigate_client.get_result().result == NavigateToPose.Result.SUCCESS:
        print("Hooray, the base moved 1 meter forward")
    else:
        print("The base failed to move forward 1 meter for some reason")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
