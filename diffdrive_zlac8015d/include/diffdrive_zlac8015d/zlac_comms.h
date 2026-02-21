#ifndef DIFFDRIVE_ZLAC8015D_ZLAC_COMMS_H
#define DIFFDRIVE_ZLAC8015D_ZLAC_COMMS_H

#include <string>
#include <modbus/modbus.h>

// ZLAC8015D Modbus Register Addresses
namespace ZlacReg {
  // Control registers
  constexpr uint16_t CONTROL_MODE    = 0x200D;  // 1=Position, 3=Velocity, 4=Torque
  constexpr uint16_t ENABLE          = 0x200E;  // 0x08=Enable, 0x07=Disable
  constexpr uint16_t CLEAR_FAULT     = 0x200F;  // Write 0x06 to clear faults

  // Velocity command registers (RPM × 10, signed int16)
  constexpr uint16_t LEFT_VEL_CMD    = 0x2088;
  constexpr uint16_t RIGHT_VEL_CMD   = 0x2089;

  // Encoder feedback registers (cumulative counts, signed int32 = 2 registers)
  constexpr uint16_t LEFT_ENCODER    = 0x20AB;
  constexpr uint16_t RIGHT_ENCODER   = 0x20AC;

  // Velocity feedback registers (RPM × 10, signed int16)
  constexpr uint16_t LEFT_VEL_FB     = 0x20AB;
  constexpr uint16_t RIGHT_VEL_FB    = 0x20AC;

  // Control mode values
  constexpr uint16_t MODE_POSITION   = 1;
  constexpr uint16_t MODE_VELOCITY   = 3;
  constexpr uint16_t MODE_TORQUE     = 4;

  // Enable/Disable values
  constexpr uint16_t MOTOR_ENABLE    = 0x08;
  constexpr uint16_t MOTOR_DISABLE   = 0x07;
}

class ZlacComms
{
public:
  ZlacComms();
  ~ZlacComms();

  /// Initialize the Modbus-RTU connection
  void setup(const std::string &serial_device, int baud_rate, int timeout_ms, int slave_id);

  /// Check if connected
  bool connected() const { return connected_; }

  /// Disconnect from device
  void disconnect();

  /// Set velocity control mode
  bool setVelocityMode();

  /// Enable both motors
  bool enable();

  /// Disable both motors
  bool disable();

  /// Clear any faults
  bool clearFault();

  /// Set motor velocities in RPM (positive = forward)
  /// The ZLAC8015D expects RPM × 10
  bool setMotorValues(int left_rpm_x10, int right_rpm_x10);

  /// Read encoder counts (cumulative)
  bool readEncoderValues(int &left_enc, int &right_enc);

private:
  modbus_t *ctx_;     ///< libmodbus context
  bool connected_;
  int slave_id_;

  /// Write a single register with error handling
  bool writeRegister(uint16_t addr, uint16_t value);

  /// Read a single holding register
  bool readRegister(uint16_t addr, uint16_t &value);
};

#endif // DIFFDRIVE_ZLAC8015D_ZLAC_COMMS_H
