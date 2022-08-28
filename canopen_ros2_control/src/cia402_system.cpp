#include "canopen_ros2_control/cia402_system.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("CIA402System");
}

namespace canopen_ros2_control
{
CIA402System::~CIA402System()
{
  executor_->cancel();
  printf("Joining...");
  spin_thread_->join();
  printf("Joined!");

  device_manager_.reset();
  executor_.reset();

  init_thread_->join();
  init_thread_.reset();

  executor_.reset();
  spin_thread_.reset();
}

return_type CIA402System::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  RCLCPP_INFO(kLogger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  RCLCPP_INFO(kLogger, "master_bin: '%s'", info_.hardware_parameters["master_bin"].c_str());
  RCLCPP_INFO(kLogger, "can_interface_name: '%s'", info_.hardware_parameters["can_interface_name"].c_str());

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_manager_ = std::make_shared<DeviceContainerNode>(executor_);
  executor_->add_node(device_manager_);

  init_thread_ = std::make_unique<std::thread>(&CIA402System::initDeviceManager, this);

  if (init_thread_->joinable())
  {
    init_thread_->join();
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return return_type::ERROR;
  }

  spin_thread_ = std::make_unique<std::thread>(&CIA402System::spin, this);

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> CIA402System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CIA402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

return_type CIA402System::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                      const std::vector<std::string>& stop_interfaces)
{
  std::vector<integration_level_t> new_modes = control_level_;
  for (std::string key : stop_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        new_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes[i] = integration_level_t::POSITION;
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != new_modes[i])
    {
      switch (new_modes[i])
      {
        case integration_level_t::POSITION:
          if (!drivers_[i]->command_mode_switch(MotorBase::Profiled_Position))
          {
            return return_type::ERROR;
          }
          break;
        case integration_level_t::VELOCITY:
          if (!drivers_[i]->command_mode_switch(MotorBase::Profiled_Velocity))
          {
            return return_type::ERROR;
          }
          break;
        case integration_level_t::EFFORT:
          break;
        case integration_level_t::UNDEFINED:
          break;
      }
    }
    control_level_[i] = new_modes[i];
  }

  return return_type::OK;
}

return_type CIA402System::start()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    drivers_[i]->start();
  }

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type CIA402System::stop()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    drivers_[i]->stop();
  }

  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type CIA402System::read()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    drivers_[i]->read(hw_states_positions_[i], hw_states_velocities_[i]);
  }

  return return_type::OK;
}

return_type CIA402System::write()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    switch (control_level_[i])
    {
      case integration_level_t::POSITION:
        if (!drivers_[i]->write(hw_commands_positions_[i]))
        {
          return return_type::ERROR;
        }
        break;
      case integration_level_t::VELOCITY:
        if (!drivers_[i]->write(hw_commands_velocities_[i]))
        {
          return return_type::ERROR;
        }
        break;
      case integration_level_t::EFFORT:
        break;
      case integration_level_t::UNDEFINED:
        break;
    }
  }

  return return_type::OK;
}

void CIA402System::initDeviceManager()
{
  std::string tmp_master_bin =
      (info_.hardware_parameters["master_bin"] == "\"\"") ? "" : info_.hardware_parameters["master_bin"];

  if (device_manager_->init(info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
                            info_.hardware_parameters["bus_config"], tmp_master_bin))
  {
    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
      drivers_.emplace_back(std::static_pointer_cast<ros2_canopen::MotionControllerDriver>(
          device_manager_->get_node(std::stoi(joint.parameters.at("node_id")))));
    }
    RCLCPP_INFO(device_manager_->get_logger(), "Initialisation successful.");
  }
  else
  {
    RCLCPP_INFO(device_manager_->get_logger(), "Initialisation failed.");
  }
}

void CIA402System::spin()
{
  executor_->spin();
  executor_->remove_node(device_manager_);

  RCLCPP_INFO(kLogger, "Exiting spin thread...");
}
}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::CIA402System, hardware_interface::SystemInterface)
