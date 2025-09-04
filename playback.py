#!/usr/bin/env python3
import time

import numpy as np
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

# --- Other Constants ---
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
PLAYBACK_INTERVAL = 0.1  # seconds
GOTO_HOME_STEPS = 100
MAX_RETRIES = 5
RETRY_DELAY = 0.01  # seconds


def write_with_retry(
    packetHandler, portHandler, joint_id, address, data, data_len_bytes
):
    """Attempts to write data to a motor with a retry mechanism."""
    for attempt in range(MAX_RETRIES):
        if data_len_bytes == 1:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, joint_id, address, data
            )
        elif data_len_bytes == 2:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
                portHandler, joint_id, address, data
            )
        elif data_len_bytes == 4:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, joint_id, address, data
            )
        else:
            print(f"Error: Invalid data length {data_len_bytes} bytes.")
            return False

        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            return True

        if attempt < MAX_RETRIES - 1:
            time.sleep(RETRY_DELAY)

    print(
        f"Error: Failed to write to ID:{joint_id}, Addr:{address} after {MAX_RETRIES} attempts."
    )
    return False


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

    # --- Enable Torque for all joints ---
    for joint_id in joint_ids:
        write_with_retry(
            packetHandler, portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, 1
        )
    print("Torque has been enabled for all arm joints.")

    # --- Load Trajectory ---
    try:
        trajectory = np.loadtxt("trajectory.csv", delimiter=",")
    except FileNotFoundError:
        print("Error: trajectory.csv not found. Please run teach.py first.")
        portHandler.closePort()
        quit()

    # --- Go to Initial Position ---
    current_positions = []
    for joint_id in joint_ids:
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
            portHandler, joint_id, ADDR_PRESENT_POSITION
        )
        current_positions.append(dxl_present_position)

    initial_positions = trajectory[0]

    print("Moving to initial position...")
    for step in range(GOTO_HOME_STEPS):
        intermediate_positions = (
            current_positions
            + (initial_positions - current_positions) * (step + 1) / GOTO_HOME_STEPS
        )
        for i, joint_id in enumerate(joint_ids):
            write_with_retry(
                packetHandler,
                portHandler,
                joint_id,
                ADDR_GOAL_POSITION,
                int(intermediate_positions[i]),
                4,
            )
        time.sleep(0.02)

    # --- Playback Logic ---
    print("\n--- Starting playback ---")
    for positions in trajectory:
        for i, joint_id in enumerate(joint_ids):
            goal_position = int(positions[i])
            write_with_retry(
                packetHandler,
                portHandler,
                joint_id,
                ADDR_GOAL_POSITION,
                goal_position,
                4,
            )

        print(f"Moving to positions: {positions.astype(int)}")
        time.sleep(PLAYBACK_INTERVAL)

    print("--- Playback finished ---")

    # --- Disable Torque after playback ---
    for joint_id in joint_ids:
        write_with_retry(
            packetHandler, portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, 1
        )

    # --- Close Port ---
    portHandler.closePort()


if __name__ == "__main__":
    main()
