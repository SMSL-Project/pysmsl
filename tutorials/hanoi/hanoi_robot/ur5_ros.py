#!/usr/bin/env python3
"""
This ROS2 node, called 'trajectory_planner', performs the following functions:
  - Subscribes to the robot’s current pose on the "current_pose" topic (type: PoseStamped).
  - Subscribes to a target pose sequence on the "target_poses" topic (type: PoseArray).
    (You can supply your array of PoseStamped by converting it or publishing as a PoseArray.)
  - Plans a trajectory from the current pose to each target pose. As the orientation is fixed,
    only the position is interpolated. A velocity parameter (and a time step) defines the step size.
  - Publishes each intermediate pose (as a PoseStamped message) on the "cmd_pose" topic.
  
Parameters:
  - velocity (float, default 0.05): The robot’s velocity used to compute each step’s distance.
  - step_time (float, default 0.1): The time interval (in seconds) between successive published steps.
  
Note:
  - This example uses a separate thread to execute the trajectory so that the callback 
    doesn’t block the ROS executor.
  - Ensure that your robot’s controller (or simulation) subscribes to the "cmd_pose" topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from math import sqrt, ceil
import threading
import time


class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Declare parameters for velocity and time step
        self.declare_parameter('velocity', 0.2)   # m/s; adjust as needed
        self.declare_parameter('step_time', 0.1)     # seconds between each step

        self.velocity = self.get_parameter('velocity').value
        self.step_time = self.get_parameter('step_time').value

        # Publisher to send intermediate goal poses to the robot's controller
        self.cmd_pose_pub = self.create_publisher(PoseStamped, '/cartesian_motion_controller/target_frame', 10)

        # Subscriber to get the current robot pose (geometry_msgs/PoseStamped)
        self.create_subscription(PoseStamped, '/cartesian_motion_controller/current_pose', self.current_pose_callback, 10)

        # Subscriber to receive the target sequence of poses as a PoseArray.
        # (You can publish an array of PoseStamped as a PoseArray, or adjust accordingly.)
        self.create_subscription(PoseArray, 'target_poses', self.target_poses_callback, 10)

        self.current_pose = None  # Stores the most recent current pose

        self.get_logger().info('Trajectory planner node initialized.')

    def current_pose_callback(self, msg: PoseStamped):
        """Callback to update the current pose."""
        self.current_pose = msg

    def target_poses_callback(self, msg: PoseArray):
        """Callback to receive the target pose sequence and start trajectory execution."""
        if self.current_pose is None:
            self.get_logger().warn('Current pose unknown; cannot plan trajectory.')
            return

        # Start processing the target sequence in a separate thread to avoid blocking the executor.
        thread = threading.Thread(target=self.execute_trajectory, args=(msg.poses,))
        thread.start()

    def execute_trajectory(self, target_poses):
        """
        For each target in the sequence:
          - Generate a list of intermediate PoseStamped messages using linear interpolation.
          - Publish each intermediate pose at intervals of self.step_time.
        """
        # Use the current position as the starting point.
        start_pose = self.current_pose.pose.position
        current_position = [start_pose.x, start_pose.y, start_pose.z]

        for target in target_poses:
            target_position = [target.position.x, target.position.y, target.position.z]

            # Generate intermediate poses from the current to the target position.
            trajectory = self.generate_interpolated_trajectory(current_position, target_position)

            for pose in trajectory:
                self.cmd_pose_pub.publish(pose)
                time.sleep(self.step_time)

            # Update the current position for the next segment.
            current_position = target_position

        self.get_logger().info('Trajectory execution complete.')

    def generate_interpolated_trajectory(self, start_pos, target_pos):
        """
        Compute a series of PoseStamped messages forming the trajectory between two positions.
        
        Args:
            start_pos (list): [x, y, z] starting position.
            target_pos (list): [x, y, z] target position.
        
        Returns:
            trajectory (list): List of PoseStamped messages representing interpolated waypoints.
        """
        dx = target_pos[0] - start_pos[0]
        dy = target_pos[1] - start_pos[1]
        dz = target_pos[2] - start_pos[2]
        distance = sqrt(dx**2 + dy**2 + dz**2)

        # Determine the step distance based on the set velocity and time step.
        step_distance = self.velocity * self.step_time

        # Calculate the number of interpolation steps (ensure at least one step).
        steps = max(1, int(ceil(distance / step_distance)))
        trajectory = []

        for i in range(1, steps + 1):
            fraction = i / steps
            intermediate_x = start_pos[0] + dx * fraction
            intermediate_y = start_pos[1] + dy * fraction
            intermediate_z = start_pos[2] + dz * fraction

            pose_msg = PoseStamped()
            # Stamp the message with the current time and a reference frame (adjust the frame_id as needed)
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = intermediate_x
            pose_msg.pose.position.y = intermediate_y
            pose_msg.pose.position.z = intermediate_z

            # Use the current orientation (assumed constant across the trajectory)
            if self.current_pose is not None:
                pose_msg.pose.orientation = self.current_pose.pose.orientation

            trajectory.append(pose_msg)

        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
