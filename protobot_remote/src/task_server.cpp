#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "protobot_interfaces/action/protobot_task.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::placeholders;

namespace protobot_remote
{

class TaskServer : public rclcpp::Node
{
public:
  using Task = protobot_interfaces::action::ProtobotTask;
  using GoalHandleTask = rclcpp_action::ServerGoalHandle<Task>;

  explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<Task>(
      this,
      "task_server",
      std::bind(&TaskServer::handle_goal, this, _1, _2),
      std::bind(&TaskServer::handle_cancel, this, _1),
      std::bind(&TaskServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Task Server has been started.");
  }

private:
    rclcpp_action::Server<Task>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::vector<double> arm_joint_goal_;
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Task::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with task number: %d", goal->task_number);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        if (arm_move_group_)
        {
            arm_move_group_->stop();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto result = std::make_shared<Task::Result>();

        if (!arm_move_group_)
        {
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        }
        

        if (goal_handle->get_goal()->task_number == 0){
            arm_joint_goal_ ={0.0, 0.0, 0.0};
        } else if (goal_handle->get_goal()->task_number == 1){
            arm_joint_goal_ = {-1.14, -0.6, -0.07};
        } else if (goal_handle->get_goal()->task_number == 2){
            arm_joint_goal_ = {-1.57, 0.0, -0.9};
        } else if (goal_handle->get_goal()->task_number == 3){
            arm_joint_goal_ = {0.0, 0.0, 0.0};
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid task number");
            return;
        }

        arm_move_group_->setStartState(*arm_move_group_->getCurrentState());

        bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);

        if (!arm_within_bounds)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal is out of bounds");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (arm_plan_success)
        {
            RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arm");
            arm_move_group_->move();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planner FAILED");
            return;
        }

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
   
};

}  // namespace protobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(protobot_remote::TaskServer);