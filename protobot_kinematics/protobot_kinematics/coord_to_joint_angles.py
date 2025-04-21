#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from protobot_interfaces.srv import Coordinates
from math import pi, acos, atan2, cos, sin

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angles')

        # Create a publisher for the '/arm_controller/joint_trajectory' topic
        self.server_ = self.create_service(Coordinates, '/coordinates', self.callback_coordinates)
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Create the JointTrajectory message
        self.trajectory_command = JointTrajectory()
        joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
        self.trajectory_command.joint_names = joint_names

    def callback_coordinates(self, request, response):
        # Extract x and y coordinates from the message
        x = request.x_coord
        y = request.y_coord

        self.get_logger().info('Received coordinates: x={}, y={}'.format(x, y))
        # Calculate joint angles using inverse kinematics
        joint_angles = self.inverse_kinematics(x, y)

        # Only proceed if joint angles are valid
        if joint_angles is not None:
            # Update the trajectory command with new joint angles
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.velocities = [0.0, 0.0, 0.0]
            point.time_from_start.sec = 2

            self.trajectory_command.points = [point]

            # Format joint angles to 2 decimal places for logging
            formatted_angles = [round(angle, 2) for angle in joint_angles]
            self.get_logger().info('Publishing joint angles: {}'.format(formatted_angles))
            
            self.publisher.publish(self.trajectory_command)

            response.success = True
        else:
            self.get_logger().info('Skipping publish due to invalid joint angles.')
            response.success = False
        
        return response

    def inverse_kinematics(self, x, y):
        # Define the length of each arm segment
        l1 = 0.05
        l2 = 0.05
        l3 = 0.05
        gamma = pi/2

        # Calculating the servo angles
        try:
            q2 = pi - acos((l1**2 + l2**2 - (x-l3*cos(gamma))**2 - (y-l3*sin(gamma))**2)/(2*l1*l2))
            q1 = atan2((y-l3*sin(gamma)), (x-l3*cos(gamma))) - atan2((l2*sin(q2)), (l1 + l2*cos(q2)))
            q3 = 0 - (q1 + q2)
            return [q1, q2, q3]
        except ValueError:
            # Handle cases where inverse kinematics fails (e.g., unreachable position)
            self.get_logger().warn('Invalid coordinates for inverse kinematics, returning empty angles.')
            return None  # Return None to indicate failure

def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()

    try:
        rclpy.spin(node)  # Spin the node to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()