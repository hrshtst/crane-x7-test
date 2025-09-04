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
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52

# --- Other Constants ---
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_RETRIES = 5
RETRY_DELAY = 0.01  # seconds
GOTO_HOME_STEPS = 100

# -- Playback Settings --
# Increase this factor to make the playback smoother by adding more intermediate points.
# A factor of 5 turns a 10Hz recording into a 50Hz playback.
INTERPOLATION_FACTOR = 5
PLAYBACK_INTERVAL = 0.02  # Corresponds to 50Hz


def write_with_retry(
    packetHandler, portHandler, joint_id, address, data, data_len_bytes
):
    """Attempts to write data to a motor with a retry mechanism and detailed error logging."""
    dxl_comm_result, dxl_error = None, None
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
            # Print detailed error on failed attempt
            print(
                f"Warning: Write failed for ID:{joint_id}, Addr:{address} (Attempt {attempt + 1}/{MAX_RETRIES})"
            )
            print(f"  - Comm Result: {packetHandler.getTxRxResult(dxl_comm_result)}")
            print(f"  - Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
            time.sleep(RETRY_DELAY)

    # Log final failure
    print(
        f"Error: Failed to write to ID:{joint_id}, Addr:{address} after {MAX_RETRIES} attempts."
    )
    print(f"  - Last Comm Result: {packetHandler.getTxRxResult(dxl_comm_result)}")
    print(f"  - Last Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
    return False


def get_joint_ids_from_config(config_file):
    """Parses the YAML config file to get all joint IDs from arm and hand groups."""
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    joint_ids = []

    # Combine joints from both 'arm' and 'hand' groups
    joint_names = (
        config["joint_groups"]["arm"]["joints"]
        + config["joint_groups"]["hand"]["joints"]
    )

    for joint_name in joint_names:
        if joint_name in config:
            joint_ids.append(config[joint_name]["id"])

    print(f"Found Joint IDs: {joint_ids}")
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
    except (FileNotFoundError, KeyError) as e:
        print(f"Error reading or parsing crane-x7.yaml: {e}")
        portHandler.closePort()
        quit()

    # **FIX**: Read the actual Min/Max position limits from each motor
    min_limits = {}
    max_limits = {}
    print("\nReading position limits from motors...")
    for joint_id in joint_ids:
        min_pos, _, _ = packetHandler.read4ByteTxRx(
            portHandler, joint_id, ADDR_MIN_POSITION_LIMIT
        )
        max_pos, _, _ = packetHandler.read4ByteTxRx(
            portHandler, joint_id, ADDR_MAX_POSITION_LIMIT
        )
        min_limits[joint_id] = min_pos
        max_limits[joint_id] = max_pos
        print(f"  - ID:{joint_id} Min:{min_pos} Max:{max_pos}")
    print("Finished reading position limits.\n")

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

    try:
        # --- Create Interpolated Trajectory ---
        interpolated_trajectory = []
        for i in range(len(trajectory) - 1):
            start_pos = trajectory[i]
            end_pos = trajectory[i + 1]
            for step in range(INTERPOLATION_FACTOR):
                interp_pos = start_pos + (end_pos - start_pos) * (
                    step / INTERPOLATION_FACTOR
                )
                interpolated_trajectory.append(interp_pos)
        interpolated_trajectory.append(trajectory[-1])  # Add the final point
        interpolated_trajectory = np.array(interpolated_trajectory)

        # --- Go to Initial Position ---
        current_positions = []
        for joint_id in joint_ids:
            dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
                portHandler, joint_id, ADDR_PRESENT_POSITION
            )
            current_positions.append(dxl_present_position)

        initial_positions = interpolated_trajectory[0]

        print("Moving to initial position...")
        for step in range(GOTO_HOME_STEPS):
            intermediate_positions = (
                np.array(current_positions)
                + (initial_positions - np.array(current_positions))
                * (step + 1)
                / GOTO_HOME_STEPS
            )
            for i, joint_id in enumerate(joint_ids):
                # **FIX**: Clamp the goal position using the actual limits read from the motor
                goal_position = int(
                    np.clip(
                        intermediate_positions[i],
                        min_limits[joint_id],
                        max_limits[joint_id],
                    )
                )
                write_with_retry(
                    packetHandler,
                    portHandler,
                    joint_id,
                    ADDR_GOAL_POSITION,
                    goal_position,
                    4,
                )
            time.sleep(0.02)

        # --- Playback Logic ---
        print("\n--- Starting playback ---")
        for positions in interpolated_trajectory:
            for i, joint_id in enumerate(joint_ids):
                # Clamp the trajectory data as well, just in case
                goal_position = int(
                    np.clip(positions[i], min_limits[joint_id], max_limits[joint_id])
                )
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

    except KeyboardInterrupt:
        print("\nPlayback stopped by user.")

    finally:
        # **FIX**: Clear the port before final commands
        portHandler.clearPort()

        for joint_id in joint_ids:
            write_with_retry(
                packetHandler,
                portHandler,
                joint_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
                1,
            )

        portHandler.closePort()
        print("Port closed.")


if __name__ == "__main__":
    main()
