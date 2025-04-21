#include "protobot_controller/protobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace protobot_controller
{

std::string compensateZeros(int value)
{
    std::string compensate_zeros = "";
    if (value < 10) {
        compensate_zeros = "00";
    } else if (value < 100) {
        compensate_zeros = "0";
    } else {
        compensate_zeros = "";
    }
    return compensate_zeros;
}

ProtobotInterface::ProtobotInterface()
{
}

ProtobotInterface::~ProtobotInterface()
{
    if (arduino_.IsOpen()) {
        try {
            arduino_.Close();
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ProtobotInterface"), "Something went wrong whilst closing the connection with port " << port_);
        }
    }
}

hardware_interface::CallbackReturn ProtobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{

    if (
        hardware_interface::SystemInterface::on_init(hardware_info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // Checks for exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("Protobot Interface"), "No serial port provided, Aborting");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Sucessfully configured command and state interfaces!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Starting the robot hardware...");
    // command and state should be equal when starting
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        set_command(name, get_state(name));
    }

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("ProtobotInterface"), "Something went wrong whilst interacting with the port " << port_);
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware started, ready to take commands");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProtobotInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{    
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Stopping the robot hardware...");
    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ProtobotInterface"), "Something went wrong whilst closing connection with the port " << port_);
            return CallbackReturn::FAILURE;
        }
        
    }
    RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Hardware stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProtobotInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        auto new_value = get_command(name);
        set_state(name, new_value);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProtobotInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{   

    // Check if all current commands match the previous ones
    bool all_match = true;
    for (const auto& name : position_interfaces_) {
        double current = get_command(name);
        // If the interface isn't in prev_position_commands_ yet (first run) or values differ
        if (prev_position_commands_.find(name) == prev_position_commands_.end() || current != prev_position_commands_[name]) {
            all_match = false;
            break;
        }
    }
    if (all_match) {
        return hardware_interface::return_type::OK;
    }

    // Construct the message string
    std::string msg;
    // Shoulder joint
    double shoulder_pos = get_command(position_interfaces_.at(0));
    int shoulder = static_cast<int>(((shoulder_pos + (M_PI/2)) * 180) / M_PI);
    msg.append("b");
    msg.append(compensateZeros(shoulder));
    msg.append(std::to_string(shoulder));
    msg.append(",");
    // Elbow joint
    double elbow_pos = get_command(position_interfaces_.at(1));
    int elbow = 180 - static_cast<int>(((elbow_pos + (M_PI/2)) * 180) / M_PI);
    msg.append("s");
    msg.append(compensateZeros(elbow));
    msg.append(std::to_string(elbow));
    msg.append(",");
    // Wrist joint
    double wrist_pos = get_command(position_interfaces_.at(2));
    int wrist = static_cast<int>(((wrist_pos + (M_PI/2)) * 180) / M_PI);
    msg.append("e");
    msg.append(compensateZeros(wrist));
    msg.append(std::to_string(wrist));
    msg.append(",");

    // Send the message
    try {
        arduino_.Write(msg);
        // For debugging, to see what values are sent
        // RCLCPP_INFO(rclcpp::get_logger("ProtobotInterface"), "Sent: %s", msg.c_str());
    } catch(...) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("ProtobotInterface"), 
            "Something went wrong whilst sending this message " << msg << " to the port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    // Update previous commands
    for (const auto& name : position_interfaces_) {
        prev_position_commands_[name] = get_command(name);
    }

    return hardware_interface::return_type::OK;
}


} // namespace protobot_controller

PLUGINLIB_EXPORT_CLASS(protobot_controller::ProtobotInterface, hardware_interface::SystemInterface);