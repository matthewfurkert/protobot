#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        point.positions = [1.0, 1.0, 1.0]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2

        self.trajectory_command.points = [point]

        # Publish the message
        self.get_logger().info('Publishing joint angles...')


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
