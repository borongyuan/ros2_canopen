#ifndef CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "canopen_402_driver/canopen_402_driver.hpp"
#include "canopen_core/device_container.hpp"
#include "canopen_ros2_control/visibility_control.h"

namespace canopen_ros2_control
{
using namespace ros2_canopen;
using hardware_interface::return_type;

class CIA402System : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  ~CIA402System();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type start() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type stop() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type read() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type write() override;

private:
  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };

  std::vector<integration_level_t> control_level_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<DeviceContainerNode> device_manager_;
  std::vector<std::shared_ptr<ros2_canopen::MotionControllerDriver>> drivers_;

  std::unique_ptr<std::thread> init_thread_;
  std::unique_ptr<std::thread> spin_thread_;

  void initDeviceManager();
  void spin();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
