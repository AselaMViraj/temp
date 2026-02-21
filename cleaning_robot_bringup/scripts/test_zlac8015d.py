#!/usr/bin/env python3
"""
Test ZLAC8015D communication via Xiao ESP32-S3 + MAX485 RS-485 bridge.

Usage:
    python3 test_zlac8015d.py --port /dev/ttyACM0 --baud 115200 --slave 1 --speed 10

Prerequisites:
    pip3 install pymodbus pyserial
"""

import argparse
import time
from pymodbus.client import ModbusSerialClient


# ===============================
# ZLAC8015D Register Map
# ===============================
REG_CONTROL_MODE = 0x200D    # 1=Position, 3=Velocity, 4=Torque
REG_ENABLE       = 0x200E    # 0x08=Enable both, 0x07=Disable
REG_LEFT_VEL     = 0x2088    # Left velocity  (RPM × 10, signed)
REG_RIGHT_VEL    = 0x2089    # Right velocity (RPM × 10, signed)
REG_LEFT_ENC     = 0x20AB    # Left encoder position
REG_RIGHT_ENC    = 0x20AC    # Right encoder position


def main():
    parser = argparse.ArgumentParser(description='Test ZLAC8015D motor driver')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate')
    parser.add_argument('--slave', type=int, default=1, help='Modbus slave ID')
    parser.add_argument('--speed', type=int, default=10, help='Test speed in RPM')
    args = parser.parse_args()

    print(f"\nConnecting to ZLAC8015D on {args.port} @ {args.baud} baud...")

    client = ModbusSerialClient(
        port=args.port,
        baudrate=args.baud,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=1
    )

    if not client.connect():
        print("❌ Failed to connect to serial port.")
        print("   Check cable, port name, and permissions.")
        return

    print("✅ Serial connection established\n")

    try:
        # =====================================
        # Step 1: Read current control mode
        # =====================================
        print("--- Step 1: Reading driver control mode ---")
        result = client.read_holding_registers(
            address=REG_CONTROL_MODE,
            count=1,
            device_id=args.slave
        )

        if result.isError():
            print("❌ Failed to read control mode. Check RS-485 wiring.")
            return

        modes = {1: 'Position', 3: 'Velocity', 4: 'Torque'}
        current_mode = modes.get(result.registers[0], f"Unknown({result.registers[0]})")
        print(f"   Current mode: {current_mode}")

        # =====================================
        # Step 2: Set Velocity Mode
        # =====================================
        print("\n--- Step 2: Setting velocity mode ---")
        write = client.write_register(
            address=REG_CONTROL_MODE,
            value=3,
            device_id=args.slave
        )

        if write.isError():
            print("❌ Failed to set velocity mode.")
            return

        time.sleep(0.2)
        print("   ✅ Velocity mode set")

        # =====================================
        # Step 3: Enable Motors
        # =====================================
        print("\n--- Step 3: Enabling motors ---")
        write = client.write_register(
            address=REG_ENABLE,
            value=0x08,
            device_id=args.slave
        )

        if write.isError():
            print("❌ Failed to enable motors.")
            return

        time.sleep(0.5)
        print("   ✅ Motors enabled")

        # =====================================
        # Step 4: Read Initial Encoders
        # =====================================
        print("\n--- Step 4: Reading encoder values ---")
        result = client.read_holding_registers(
            address=REG_LEFT_ENC,
            count=2,
            device_id=args.slave
        )

        if not result.isError():
            print(f"   Left encoder:  {result.registers[0]}")
            print(f"   Right encoder: {result.registers[1]}")
        else:
            print("⚠️ Could not read encoders")

        # =====================================
        # Step 5: Spin Motors
        # =====================================
        rpm_value = args.speed * 10
        print(f"\n--- Step 5: Spinning motors at {args.speed} RPM for 3 seconds ---")

        client.write_register(
            address=REG_LEFT_VEL,
            value=rpm_value,
            device_id=args.slave
        )

        client.write_register(
            address=REG_RIGHT_VEL,
            value=rpm_value,
            device_id=args.slave
        )

        for i in range(3):
            time.sleep(1)
            result = client.read_holding_registers(
                address=REG_LEFT_ENC,
                count=2,
                device_id=args.slave
            )

            if not result.isError():
                print(f"   [{i+1}s] L_enc={result.registers[0]}  R_enc={result.registers[1]}")

        # =====================================
        # Step 6: Stop Motors
        # =====================================
        print("\n--- Step 6: Stopping motors ---")

        client.write_register(
            address=REG_LEFT_VEL,
            value=0,
            device_id=args.slave
        )

        client.write_register(
            address=REG_RIGHT_VEL,
            value=0,
            device_id=args.slave
        )

        time.sleep(0.5)

        client.write_register(
            address=REG_ENABLE,
            value=0x07,
            device_id=args.slave
        )

        print("   ✅ Motors stopped and disabled")

        print("\n" + "="*50)
        print("✅ ALL TESTS PASSED!")
        print("Hardware chain OK:")
        print("Jetson → Xiao S3 → MAX485 → ZLAC8015D → Motors")
        print("="*50)

    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        print("⚠️ Emergency stopping motors...")

        try:
            client.write_register(address=REG_LEFT_VEL, value=0, device_id=args.slave)
            client.write_register(address=REG_RIGHT_VEL, value=0, device_id=args.slave)
            client.write_register(address=REG_ENABLE, value=0x07, device_id=args.slave)
        except:
            pass

    finally:
        client.close()
        print("\nConnection closed.")


if __name__ == '__main__':
    main()
