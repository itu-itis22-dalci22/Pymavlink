import serial.tools.list_ports
import time
from pymavlink_helper import PyMavlinkHelper


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def prompt_disarm(helper: PyMavlinkHelper, drone_index):
    """Function to prompt the user to disarm a specific drone."""
    while True:
        altitude = helper.get_current_state()[drone_index][2]
        if altitude < 0.5:
            print(f"Drone {drone_index}: Altitude is below 3 meters.")
            disarm_command = (
                input(f"Do you want to disarm drone {drone_index} now? (yes/no): ")
                .strip()
                .lower()
            )
            if disarm_command == "yes":
                helper.close_environment()
                print(f"Drone {drone_index} disarmed.")
                break
            elif disarm_command == "no":
                print(f"Drone {drone_index} is not disarmed.")
            else:
                print("Please enter 'yes' or 'no'.")
        time.sleep(3)  # Wait for 3 seconds before asking again


def main():
    print("Available serial ports:")
    ports = list_serial_ports()
    baud_rate = 57600
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    selected_ports = input(
        "Enter the indices of the serial ports to use, separated by commas: "
    )
    try:
        selected_ports = [
            ports[int(index.strip())] for index in selected_ports.split(",")
        ]
    except (ValueError, IndexError) as e:
        print("Invalid port index. Please enter valid indices separated by commas.")
        return

    drone_count = len(selected_ports)
    helper = PyMavlinkHelper(drone_count, selected_ports, baud_rate)

    try:
        helper.initialize_environment()

        while True:
            command = (
                input("Enter command (arm, disarm, takeoff, land, move, quit): ")
                .strip()
                .lower()
            )

            if command == "arm":
                helper._arm_drones()

            elif command == "disarm":
                helper.close_environment()

            elif command == "takeoff":
                try:
                    altitude = float(input("Enter the target altitude in meters: "))
                    helper.takeoff(altitude)

                    # Continuously display altitude during takeoff
                    while True:
                        current_altitude = helper.get_current_state()[0][
                            2
                        ]  # Assuming only one drone
                        print(f"Current altitude: {current_altitude:.2f} meters")
                        if current_altitude >= altitude:
                            print(f"Reached target altitude of {altitude:.2f} meters.")
                            break
                        time.sleep(1)  # Update every second

                except ValueError:
                    print("Invalid altitude. Please enter a numeric value.")

            elif command == "land":
                helper.land_drones()

                # Monitor each drone's altitude for disarming
                for i in range(drone_count):
                    while True:
                        current_altitude = helper.get_current_state()[i][2]
                        print(f"Drone {i} altitude: {current_altitude:.2f} meters")
                        if (
                            current_altitude < 1
                        ):  # Assuming landing is complete when below 1 meter
                            break
                        time.sleep(1)  # Update every second
                    prompt_disarm(helper, i)

            elif command == "move":
                coords_input = input(
                    "Enter the target coordinates for each drone (format: x,y,z; x,y,z; ...): "
                )
                try:
                    coords = [
                        tuple(map(float, coord.split(",")))
                        for coord in coords_input.split(";")
                    ]
                    if len(coords) != drone_count:
                        print(
                            "Error: Number of coordinates does not match number of drones."
                        )
                        continue
                    helper.move(coords)

                    while True:
                        start_time = time.time()
                        print("Drones are moving...")

                        while time.time() - start_time < 5:
                            states = helper.get_current_state()
                            for i, state in enumerate(states):
                                print(
                                    f"Drone {i} position: x={state[0]:.2f}, y={state[1]:.2f}, z={state[2]:.2f}"
                                )
                            time.sleep(1)  # Update every second

                        land_command = (
                            input("Do you want to land the drones now? (yes/no): ")
                            .strip()
                            .lower()
                        )
                        if land_command == "yes":
                            helper.land_drones()
                            break
                        elif land_command == "no":
                            pass
                        else:
                            print("Please enter 'yes' or 'no'.")

                except ValueError:
                    print(
                        "Invalid coordinates. Please enter coordinates in the format: x,y,z; x,y,z; ..."
                    )

            elif command == "quit":
                break

            else:
                print(
                    "Unknown command. Please enter one of the following: arm, disarm, takeoff, land, move, quit."
                )

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Closing environment...")

    finally:
        # Ensure that connections are closed properly
        helper.close_environment()


if __name__ == "__main__":
    main()
