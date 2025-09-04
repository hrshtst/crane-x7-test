#!/usr/bin/env python3
import time

import yaml
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# --- Dynamixel Settings ---
PROTOCOL_VERSION = 2.0
BAUDRATE = 3000000
DEVICENAME = "/dev/ttyUSB0"

# --- Control Table Addresses (for XM430 and XM540) ---
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_POSITION_P_GAIN = 84
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_D_GAIN = 80


# --- Other Constants ---
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
RECORDING_INTERVAL = 0.1  # seconds

# -- PID GAINS --
# A very low P-gain makes the robot compliant and easy to move by hand.
# This value can be tuned to your preference for more or less resistance.
P_GAIN_TEACH = 5
# Default P-gain for the CRANE-X7, used for playback.
P_GAIN_PLAYBACK = 800
I_GAIN = 0
D_GAIN = 0


def get_joint_ids_from_config(config_file):
    """Parses the YAML config file to get joint IDs."""
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    joint_ids = []
    for joint_name in config["joint_groups"]["arm"]["joints"]:
        joint_ids.append(config[joint_name]["id"])
    return joint_ids


def main():
    # --- Initialize Port and Packet Handlers ---
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # --- Open Port ---
    if not portHandler.openPort():
        print("Failed to open the port")
        quit()
    print("Succeeded to open the port")

    # --- Set Baudrate ---
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        quit()
    print("Succeeded to change the baudrate")

    # --- Get Joint IDs ---
    try:
        joint_ids = get_joint_ids_from_config("crane-x7.yaml")
    except FileNotFoundError:
        print("Error: crane-x7.yaml not found. Please place it in the same directory.")
        portHandler.closePort()
        quit()

    # --- Enable Torque and set low PID gains for compliant teaching ---
    for joint_id in joint_ids:
        packetHandler.write1ByteTxRx(
            portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        packetHandler.write2ByteTxRx(
            portHandler, joint_id, ADDR_POSITION_P_GAIN, P_GAIN_TEACH
        )
        packetHandler.write2ByteTxRx(
            portHandler, joint_id, ADDR_POSITION_I_GAIN, I_GAIN
        )
        packetHandler.write2ByteTxRx(
            portHandler, joint_id, ADDR_POSITION_D_GAIN, D_GAIN
        )

    print("Torque enabled with low PID gains. You can now move the robot.")
    print("Press Ctrl+C to stop recording and save.")

    # --- Recording Logic ---
    trajectory = []
    try:
        while True:
            joint_positions = []
            for joint_id in joint_ids:
                dxl_present_position, dxl_comm_result, dxl_error = (
                    packetHandler.read4ByteTxRx(
                        portHandler, joint_id, ADDR_PRESENT_POSITION
                    )
                )
                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    joint_positions.append(dxl_present_position)
                else:
                    joint_positions.append(-1)  # Placeholder for failed read

            trajectory.append(joint_positions)
            print(f"Recorded positions: {joint_positions}")
            time.sleep(RECORDING_INTERVAL)

    except KeyboardInterrupt:
        print("\nRecording stopped.")

    finally:
        # --- Restore default P-gain and disable Torque ---
        for joint_id in joint_ids:
            packetHandler.write2ByteTxRx(
                portHandler, joint_id, ADDR_POSITION_P_GAIN, P_GAIN_PLAYBACK
            )
            packetHandler.write1ByteTxRx(
                portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )

        # --- Save Trajectory ---
        if trajectory:
            with open("trajectory.csv", "w") as f:
                for positions in trajectory:
                    f.write(",".join(map(str, positions)) + "\n")
            print("Trajectory saved to trajectory.csv")

        # --- Close Port ---
        portHandler.closePort()


if __name__ == "__main__":
    main()
