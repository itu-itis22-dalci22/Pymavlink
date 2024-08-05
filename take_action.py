import serial.tools.list_ports
from pymavlink import mavutil
import pymavlink
import pymavlink_helper

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports

def set_mode(drone: mavutil.mavlink_connection, mode: str) -> None:
    """
    Set the flight mode of the drone.
    """
    if mode not in drone.mode_mapping():
        print(f"Unknown mode: {mode}")
        print(f"Available modes: {list(drone.mode_mapping().keys())}")
        return

    mode_id = drone.mode_mapping()[mode]
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

def main():
    print("Available serial ports:")
    ports = list_serial_ports()
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    selected_ports = input("Enter the indices of the serial ports to use, separated by commas: ")
    selected_ports = [ports[int(index.strip())] for index in selected_ports.split(",")]

    baud_rate = int(input("Enter the baud rate (e.g., 57600): "))

    drone_count = len(selected_ports)
    helper = pymavlink_helper(drone_count, selected_ports, baud_rate)
    helper.initialize_environment()

    while True:
        command = input("Enter command (arm, disarm, takeoff, land, move, quit): ").strip().lower()

        if command == "arm":
            helper._arm_drones()

        elif command == "disarm":
            helper.close_environment()

        elif command == "takeoff":
            altitude = float(input("Enter the target altitude in meters: "))
            helper.takeoff(altitude)

        elif command == "land":
            helper.land_drones()

        elif command == "move":
            coords_input = input("Enter the target coordinates for each drone (format: x,y,z; x,y,z; ...): ")
            coords = [tuple(map(float, coord.split(','))) for coord in coords_input.split(';')]
            if len(coords) != drone_count:
                print("Error: Number of coordinates does not match number of drones.")
                continue
            helper.move(coords)

        elif command == "quit":
            helper.close_environment()
            break

        else:
            print("Unknown command. Please enter one of the following: arm, disarm, takeoff, land, move, quit.")

if __name__ == "__main__":
    main()
