#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // Set the target joint position for the arm (goal)
    std::vector<double> arm_joint_goal {1.57, 0.0, 0.0};

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);

    if (!arm_within_bounds) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Target joint position(s) outside of limits");
        return;
    }

    // Plans the trajectory to reached desired goal
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Executes the planned trajectory
    if (arm_plan_success){
        arm_move_group.move();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
        return;
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("SimpleMoveitInterface");
    move_robot(node);
    rclcpp::shutdown();
    return 0;
}
