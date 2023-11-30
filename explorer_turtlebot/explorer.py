#!/usr/bin/env python3
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

# Import necessary libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from random import randrange
import time
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

class Explore(Node):

    def __init__(self):
        super().__init__('explore')
        self.get_logger().info("Explore node has been started")
        # Create an ActionClient for the NavigateToPose action
        self.navigate_to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigate_to_pose.wait_for_server()
        self.get_logger().debug("NavigateToPose is ready") 

        # Initialize variables
        self.x = 0
        self.y = 0
        self.completion = True
        self.robot_x = 0
        self.robot_y = 0

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Create a subscriber for the /map topic
        self.map = OccupancyGrid()
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.count = 0


    def map_callback(self, data):
        # This function is called whenever a new map is published
        valid = False

        while valid is False:
            # Randomly select a cell in the map
            map_random_cell = randrange(0, len(data.data))
            self.map = data.data[map_random_cell]

            # Check if the selected cell and its neighbors are valid
            edges = self.check_neighbors(data, map_random_cell)
            if self.map != -1 and self.map <= 0.2 and edges is True:
                valid = True
        
        # Convert the selected cell's index to world coordinates
        row = map_random_cell / data.info.width
        col = map_random_cell % data.info.width

        self.get_logger().info(f"map coordinates: {row, col}")

         # Convert the row and column indices to world coordinates
        self.x = col * data.info.resolution + data.info.origin.position.x
        self.y = row * data.info.resolution + data.info.origin.position.y

        
        # If the completion count is even, increment it and set a new goal
        if self.completion:
            self.set_goal()
    

    def set_goal(self):
        # This function sets a new goal for the robot
        self.get_logger().info("Setting goal")

        # This function is called whenever feedback is received from the NavigateToPose action server
        self.completion = False
        # Create a new goal
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.orientation.w = 1.0
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"goal: {goal.pose.pose.position.x, goal.pose.pose.position.y}")
        self.get_logger().info(f"robot: {self.robot_x, self.robot_y}")
        # Send the goal to the NavigateToPose action server
        self.navigate_to_pose.send_goal_async(goal, feedback_callback=self.goal_status)

    
    def goal_response_callback(self, future):
        self.get_logger().info("Goal response received")


    def goal_status(self, feedback):

        distance_remaining = feedback.feedback.distance_remaining
        recoveries = feedback.feedback.number_of_recoveries
        self.get_logger().info(f"Distance remaining: {distance_remaining}")
        # Check if the goal is reached
        if distance_remaining < 0.5:
            self.get_logger().info("Goal succeeded")
            self.completion = True
        
        if recoveries > 0:
            self.get_logger().info("Goal failed")
            self.completion = True
        
        # You can add more conditions here based on the feedback
        # For example, you might want to handle cases where the robot is stuck and the distance remaining is not changing


    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def check_neighbors(self, data, map_random_cell):
        # This function checks the neighbors of a cell in the map
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = y * data.info.width  + x
                try:
                    # Count the number of unknown and obstacle cells
                    if data.data[map_random_cell + row] == -1:
                        unknowns += 1
                    elif data.data[map_random_cell + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        # If there are unknown cells and less than two obstacle cells, the cell is valid
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False


def main(args=None):
    # Initialize the node and spin it
    rclpy.init(args=args)
    explore = Explore()
    rclpy.spin(explore)


if __name__ == '__main__':
    main()