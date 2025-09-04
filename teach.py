import os
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

# --- Other Constants ---
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
RECORDING_INTERVAL = 0.1  # seconds


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

    # --- Disable Torque for all joints ---
    for joint_id in joint_ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"ID:{joint_id} {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"ID:{joint_id} {packetHandler.getRxPacketError(dxl_error)}")
    print("Torque has been disabled for all arm joints. You can now move the robot.")

    # --- Recording Logic ---
    trajectory = []
    is_recording = False
    print("\nPress 's' to start recording, 'q' to quit and save.")

    while True:
        # A simple key press detection
        if os.name == "nt":
            import msvcrt

            if msvcrt.kbhit():
                key = msvcrt.getch().decode()
                if key == "s":
                    is_recording = True
                    print("--- Recording started ---")
                elif key == "q":
                    break
        else:
            import sys
            import termios
            import tty

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
                if ch == "s":
                    is_recording = True
                    print("--- Recording started ---")
                elif ch == "q":
                    break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if is_recording:
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
                    # Append a placeholder if reading fails
                    joint_positions.append(-1)

            trajectory.append(joint_positions)
            print(f"Recorded positions: {joint_positions}")
            time.sleep(RECORDING_INTERVAL)

    # --- Enable Torque after recording ---
    for joint_id in joint_ids:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, joint_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
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
