#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi, acos, atan2, cos, sin

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angles')

        # Create a publisher for the '/arm_controller/joint_trajectory' topic
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Create the JointTrajectory message
        self.trajectory_command = JointTrajectory()
        joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
        self.trajectory_command.joint_names = joint_names

        point = JointTrajectoryPoint()
        #['shoulder_joint', 'elbow_joint', 'wrist_joint']
        joint_angles = self.inverse_kinematics(0.07, 0.1)
        point.positions = joint_angles
        # point.positions = [1.0, 1.0, 1.0]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2

        self.trajectory_command.points = [point]

        # Publish the message
        self.get_logger().info('Publishing joint angles...')
    
    def inverse_kinematics(self, x, y):
        # Define the length of each arm segment
        l1 = 0.05
        l2 = 0.05
        l3 = 0.05
        gamma = pi/2

        # Calculating the servo angles
        q2 = pi - acos((l1**2 + l2**2 - (x-l3*cos(gamma))**2 - (y-l3*sin(gamma))**2)/(2*l1*l2))
        q1 = atan2((y-l3*sin(gamma)),(x-l3*cos(gamma))) - atan2((l2*sin(q2)),(l1 + l2*cos(q2)))
        q3 = 0 - (q1+q2)
        return q1, q2, q3


    def send_joint_angles(self):

        while rclpy.ok():
            self.publisher.publish(self.trajectory_command)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()

    try:
        node.send_joint_angles()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
