#include "diffdrive_zlac8015d/zlac_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <unistd.h>

static rclcpp::Logger logger = rclcpp::get_logger("ZlacComms");

ZlacComms::ZlacComms()
    : ctx_(nullptr), connected_(false), slave_id_(1)
{
}

ZlacComms::~ZlacComms()
{
  disconnect();
}

void ZlacComms::setup(const std::string &serial_device, int baud_rate, int timeout_ms, int slave_id)
{
  slave_id_ = slave_id;

  // Create Modbus-RTU context
  ctx_ = modbus_new_rtu(serial_device.c_str(), baud_rate, 'N', 8, 1);
  if (ctx_ == nullptr) {
    RCLCPP_ERROR(logger, "Failed to create Modbus context: %s", modbus_strerror(errno));
    return;
  }

  // Set slave ID
  modbus_set_slave(ctx_, slave_id_);

  // Set response timeout
  modbus_set_response_timeout(ctx_, timeout_ms / 1000, (timeout_ms % 1000) * 1000);

  // Enable debug mode for troubleshooting (disable in production)
  // modbus_set_debug(ctx_, TRUE);

  // Connect
  if (modbus_connect(ctx_) == -1) {
    RCLCPP_ERROR(logger, "Modbus connection failed on %s: %s",
                 serial_device.c_str(), modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return;
  }

  connected_ = true;
  RCLCPP_INFO(logger, "Connected to ZLAC8015D on %s @ %d baud (slave %d)",
              serial_device.c_str(), baud_rate, slave_id_);

  // Small delay to let the connection stabilize
  usleep(100000);  // 100ms
}

void ZlacComms::disconnect()
{
  if (ctx_ != nullptr) {
    // Try to disable motors before disconnecting
    if (connected_) {
      disable();
    }
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
    connected_ = false;
    RCLCPP_INFO(logger, "Disconnected from ZLAC8015D");
  }
}

bool ZlacComms::setVelocityMode()
{
  RCLCPP_INFO(logger, "Setting velocity control mode");
  return writeRegister(ZlacReg::CONTROL_MODE, ZlacReg::MODE_VELOCITY);
}

bool ZlacComms::enable()
{
  RCLCPP_INFO(logger, "Enabling motors");
  return writeRegister(ZlacReg::ENABLE, ZlacReg::MOTOR_ENABLE);
}

bool ZlacComms::disable()
{
  RCLCPP_INFO(logger, "Disabling motors");
  // Stop motors first
  setMotorValues(0, 0);
  usleep(50000);  // 50ms
  return writeRegister(ZlacReg::ENABLE, ZlacReg::MOTOR_DISABLE);
}

bool ZlacComms::clearFault()
{
  RCLCPP_INFO(logger, "Clearing faults");
  return writeRegister(ZlacReg::CLEAR_FAULT, 0x06);
}

bool ZlacComms::setMotorValues(int left_rpm_x10, int right_rpm_x10)
{
  if (!connected_) return false;

  // Write left velocity
  // Note: ZLAC8015D uses signed int16 for velocity
  // Negative = reverse direction
  uint16_t left_val = static_cast<uint16_t>(static_cast<int16_t>(left_rpm_x10));
  uint16_t right_val = static_cast<uint16_t>(static_cast<int16_t>(right_rpm_x10));

  bool ok = writeRegister(ZlacReg::LEFT_VEL_CMD, left_val);
  ok = ok && writeRegister(ZlacReg::RIGHT_VEL_CMD, right_val);

  return ok;
}

bool ZlacComms::readEncoderValues(int &left_enc, int &right_enc)
{
  if (!connected_) return false;

  uint16_t left_val = 0, right_val = 0;
  bool ok = readRegister(ZlacReg::LEFT_ENCODER, left_val);
  ok = ok && readRegister(ZlacReg::RIGHT_ENCODER, right_val);

  if (ok) {
    // Convert uint16 to signed int16 (encoder values can be negative)
    left_enc = static_cast<int>(static_cast<int16_t>(left_val));
    right_enc = static_cast<int>(static_cast<int16_t>(right_val));
  }

  return ok;
}

bool ZlacComms::writeRegister(uint16_t addr, uint16_t value)
{
  if (ctx_ == nullptr || !connected_) {
    RCLCPP_ERROR(logger, "Cannot write register: not connected");
    return false;
  }

  int rc = modbus_write_register(ctx_, addr, value);
  if (rc == -1) {
    RCLCPP_ERROR(logger, "Failed to write register 0x%04X = 0x%04X: %s",
                 addr, value, modbus_strerror(errno));
    return false;
  }
  return true;
}

bool ZlacComms::readRegister(uint16_t addr, uint16_t &value)
{
  if (ctx_ == nullptr || !connected_) {
    RCLCPP_ERROR(logger, "Cannot read register: not connected");
    return false;
  }

  int rc = modbus_read_registers(ctx_, addr, 1, &value);
  if (rc == -1) {
    RCLCPP_ERROR(logger, "Failed to read register 0x%04X: %s",
                 addr, modbus_strerror(errno));
    return false;
  }
  return true;
}
