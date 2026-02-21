#include "diffdrive_zlac8015d/diffdrive_zlac8015d.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>


DiffDriveZlac8015d::DiffDriveZlac8015d()
    : logger_(rclcpp::get_logger("DiffDriveZlac8015d"))
{}


hardware_interface::CallbackReturn DiffDriveZlac8015d::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring ZLAC8015D hardware interface...");

  time_ = std::chrono::system_clock::now();

  // Read parameters from URDF ros2_control tag
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // ZLAC8015D-specific parameter
  if (info_.hardware_parameters.count("slave_id")) {
    cfg_.slave_id = std::stoi(info_.hardware_parameters["slave_id"]);
  }

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Configuration complete:");
  RCLCPP_INFO(logger_, "  Device: %s @ %d baud", cfg_.device.c_str(), cfg_.baud_rate);
  RCLCPP_INFO(logger_, "  Slave ID: %d", cfg_.slave_id);
  RCLCPP_INFO(logger_, "  Encoder CPR: %d", cfg_.enc_counts_per_rev);
  RCLCPP_INFO(logger_, "  Left wheel: %s", cfg_.left_wheel_name.c_str());
  RCLCPP_INFO(logger_, "  Right wheel: %s", cfg_.right_wheel_name.c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveZlac8015d::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveZlac8015d::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn DiffDriveZlac8015d::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating ZLAC8015D controller...");

  // Connect to the ZLAC8015D via Modbus-RTU
  zlac_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout, cfg_.slave_id);

  if (!zlac_.connected()) {
    RCLCPP_ERROR(logger_, "Failed to connect to ZLAC8015D!");
    return CallbackReturn::ERROR;
  }

  // Clear any previous faults
  zlac_.clearFault();
  usleep(100000);  // 100ms delay

  // Set velocity control mode
  if (!zlac_.setVelocityMode()) {
    RCLCPP_ERROR(logger_, "Failed to set velocity mode!");
    return CallbackReturn::ERROR;
  }
  usleep(50000);  // 50ms delay

  // Enable motors
  if (!zlac_.enable()) {
    RCLCPP_ERROR(logger_, "Failed to enable motors!");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "ZLAC8015D controller activated successfully!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveZlac8015d::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating ZLAC8015D controller...");

  zlac_.disable();
  zlac_.disconnect();

  RCLCPP_INFO(logger_, "ZLAC8015D controller deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveZlac8015d::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!zlac_.connected())
  {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "ZLAC8015D not connected during read()");
    return return_type::ERROR;
  }

  // Read encoder values from ZLAC8015D via Modbus
  if (!zlac_.readEncoderValues(l_wheel_.enc, r_wheel_.enc))
  {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                         "Failed to read encoder values");
    return return_type::OK;  // Don't fail hard on a single read miss
  }

  // Calculate position (radians) from encoder counts
  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveZlac8015d::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!zlac_.connected())
  {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "ZLAC8015D not connected during write()");
    return return_type::ERROR;
  }

  // Convert velocity command from rad/s to RPM × 10
  // rad/s → RPM: multiply by 60/(2π)
  // RPM → RPM×10: multiply by 10
  // Combined factor: 60 * 10 / (2π) ≈ 95.493
  double left_rpm_x10 = l_wheel_.cmd * 60.0 * 10.0 / (2.0 * M_PI);
  double right_rpm_x10 = r_wheel_.cmd * 60.0 * 10.0 / (2.0 * M_PI);

  // Clamp to int16 range
  int left_val = std::max(-32767, std::min(32767, static_cast<int>(left_rpm_x10)));
  int right_val = std::max(-32767, std::min(32767, static_cast<int>(right_rpm_x10)));

  zlac_.setMotorValues(left_val, right_val);

  return return_type::OK;
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveZlac8015d,
  hardware_interface::SystemInterface
)
