import serial.tools.list_ports
import time
from pymavlink_helper import PyMavlinkHelper


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    return available_ports


def main():
    print("Available serial ports:")
    ports = list_serial_ports()
    baud_rate = 57600
    if not ports:
        print("No available serial ports found. Exiting...")
        return

    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    selected_ports = input(
        "Enter the indices of the serial ports to use, separated by commas: "
    )
    try:
        selected_ports = [
            ports[int(index.strip())] for index in selected_ports.split(",")
        ]
    except (ValueError, IndexError):
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

                except Exception as e:
                    print(f"Error during takeoff: {e}")
                    print("Attempting to land drones...")
                    try:
                        helper.land_drones()
                    except Exception as e:
                        print(f"Error during emergency landing: {e}")

            elif command == "land":
                try:
                    helper.land_drones()
                except Exception as e:
                    print(f"Error during landing: {e}")
                    print("Attempting to continue landing...")

                # Monitor each drone's altitude for disarming
                for i in range(drone_count):
                    try:
                        while True:
                            current_altitude = helper.get_current_state()[i][2]
                            print(f"Drone {i} altitude: {current_altitude:.2f} meters")
                            if current_altitude < 1:
                                print(f"Drone {i} has landed successfully.")
                                break
                            time.sleep(1)  # Update every second
                    except Exception as e:
                        print(f"Error monitoring altitude for Drone {i}: {e}")
                        print("Continuing to monitor other drones...")

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
                            try:
                                helper.land_drones()
                            except Exception as e:
                                print(f"Error during landing: {e}")
                                print("Attempting to continue landing...")
                            break
                        elif land_command == "no":
                            pass
                        else:
                            print("Please enter 'yes' or 'no'.")

                except Exception as e:
                    print(f"Error during move operation: {e}")
                    print("Attempting to land drones...")
                    try:
                        helper.land_drones()
                    except Exception as e:
                        print(f"Error during emergency landing: {e}")

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
