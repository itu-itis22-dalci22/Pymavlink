from pymavlink import mavutil
import time
import threading
from pyproj import Transformer, CRS
from typing import Tuple, List, Dict
from utils import (
    lla_to_xyz,
    xyz_to_lla,
    calculate_average_coordinates,
    send_position_target_global_int,
    set_parameter,
    try_recv_match,
)
import math
import csv
from datetime import datetime
import os


class PyMavlinkHelper:
    """
    PyMavlink environment helper class that provides a high-level interface to interact with the environment.
    """

    def __init__(
        self,
        drone_count: int,
        connection_strings: List[str],
        baud_rate: int,
        log_coords_file: str = "flight_coord_log.csv",
        message_log_file: str = "message_log.csv",
    ) -> None:
        """
        Initializes the PyMavlinkHelper class.

        This class is designed to manage multiple drones using pymavlink, providing
        an interface for connecting, logging, and controlling the drones.

        Parameters
        ----------
        drone_count : int
            The number of drones to be managed.
        connection_strings : List[str]
            List of connection strings for each drone. Each connection string
            should correspond to a specific drone.
        baud_rate : int
            Baud rate for the serial communication with the drones.
        log_coords_file : str, optional
            The file name for logging the coordinates of the drones during flight
            (default is 'flight_coord_log.csv').
        message_log_file : str, optional
            The file name for logging messages (default is 'message_log.csv').

        Attributes
        ----------
        drone_count : int
            Stores the number of drones.
        connection_strings : List[str]
            Stores the connection strings for each drone.
        baud_rate : int
            Stores the baud rate for serial communication.
        log_coords_file : str
            The file name for logging flight coordinates.
        message_log_file : str
            The file name for logging messages.
        vehicles : List
            A list that will hold the connected drone vehicle objects.
        is_initialized_env : bool
            A flag indicating whether the environment has been initialized.
        """
        self.drone_count = drone_count
        self.connection_strings = connection_strings
        self.baud_rate = baud_rate
        self.log_coords_file = log_coords_file
        self.message_log_file = message_log_file
        self.vehicles = []
        self.is_initialized_env = False

    def _set_mode(self, drone: mavutil.mavlink_connection, mode: str) -> None:
        """
        Sets the flight mode of the drone.

        This function attempts to change the flight mode of a drone by sending the appropriate
        MAVLink command. It waits for an acknowledgment from the drone to confirm the mode change.

        Parameters
        ----------
        drone : mavutil.mavlink_connection
            The MAVLink connection object representing the drone.
        mode : str
            The desired flight mode (e.g., "GUIDED", "LOITER", "RTL").

        Raises
        ------
        ValueError
            If the specified mode is unknown or the mode change is not accepted by the drone.
        RuntimeError
            If no acknowledgment is received or an error occurs during the mode change process.
        """

        # Get the mode ID
        if mode not in drone.mode_mapping():
            print(f"Unknown mode: {mode}")
            print(f"Available modes: {list(drone.mode_mapping().keys())}")
            return  # TODO assert exception here

        mode_id = drone.mode_mapping()[mode]

        # Set the mode
        drone.mav.set_mode_send(
            drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        # Wait for ACK command
        # MAVLink requires an ACK from the drone to confirm the mode change
        ack = None
        while not ack:
            ack = try_recv_match(drone, message_name="COMMAND_ACK")
            if ack:
                try:
                    ack_result = ack.result
                    if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"Mode change to {mode} accepted")
                        else:
                            print(
                                f"Mode change to {mode} failed with result {ack_result}"  # TODO assert exception here
                            )
                        break
                except AttributeError as e:
                    print(
                        f"Error processing ACK message: {e}"
                    )  # TODO assert exception here
            else:
                print("No ACK received, retrying...")

    def initialize_environment(self) -> None:
        """
        Connects to all drones using the provided connection strings and baud rate.

        This method establishes connections to the drones, sets each drone to the "LOITER"
        flight mode, and initializes the logging thread. It also calculates the average
        coordinates of the drones' initial positions to set the origin point.

        Raises
        ------
        ConnectionError
            If any drone fails to connect or send a heartbeat signal.
        """
        for i in range(self.drone_count):
            print(
                f"Connecting to vehicle {i+1} on: {self.connection_strings[i]} with baud rate: {self.baud_rate}"
            )
            vehicle = mavutil.mavlink_connection(
                self.connection_strings[i], baud=self.baud_rate
            )
            vehicle.wait_heartbeat()
            self.vehicles.append(vehicle)
            time.sleep(0.1)
            self._set_mode(vehicle, "LOITER")

        initial_coords = []
        for drone_index, _ in enumerate(self.vehicles):
            coords = self._get_smoothed_location(drone_index)
            initial_coords.append(coords)

        self.origin = calculate_average_coordinates(initial_coords)
        self.is_initialized_env = True
        self.logging_thread = threading.Thread(target=self._log_flight)
        self.logging_thread.start()

        print("All vehicles connected.")
        print(f"Origin is {self.origin}")

    def _log_flight(self):
        """
        Logs the flight data of the drones to CSV files.

        This method logs the current coordinates and MAVLink messages of all connected drones
        to separate CSV files in specified directories. Coordinates and messages are logged
        every second.

        The log files are specified by `log_coords_file` and `message_log_file` attributes.
        Each entry in the coordinate log includes the timestamp, drone ID, latitude, longitude,
        and altitude. The message log includes the timestamp, drone ID, message type, and message content.

        Raises
        ------
        IOError
            If there is an issue writing to the log files.
        """
        # Define directories for logs
        coords_dir = "logs/coords"
        messages_dir = "logs/messages"

        # Create directories if they do not exist
        os.makedirs(coords_dir, exist_ok=True)
        os.makedirs(messages_dir, exist_ok=True)

        # Define timestamp for unique file names
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Define log file paths with timestamp
        coords_file_path = os.path.join(
            coords_dir, f"{self.log_coords_file}_{timestamp}.csv"
        )
        message_file_path = os.path.join(
            messages_dir, f"{self.message_log_file}_{timestamp}.csv"
        )

        try:
            with open(coords_file_path, mode="a", newline="") as coords_file, open(
                message_file_path, mode="a", newline=""
            ) as message_file:

                coord_writer = csv.writer(coords_file)
                message_writer = csv.writer(message_file)

                # Write header rows if files are empty
                if os.stat(coords_file_path).st_size == 0:
                    coord_writer.writerow(
                        ["Timestamp", "DroneID", "Latitude", "Longitude", "Altitude"]
                    )

                if os.stat(message_file_path).st_size == 0:
                    message_writer.writerow(["Timestamp", "DroneID", "Message"])

                while self.is_initialized_env:
                    drone_coords = self.get_current_state()
                    for i, drone in enumerate(self.vehicles):
                        # Log coordinates
                        x, y, z = drone_coords[i]
                        x, y, z = round(x, 2), round(y, 2), round(z, 2)
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                        coord_writer.writerow([timestamp, i + 1, x, y, z])

                    # Log MAVLink messages for each drone
                    for i, drone in enumerate(self.vehicles):
                        msg = drone.recv_match(blocking=True)
                        if msg:
                            message_type = msg.get_type()
                            message_content = str(
                                msg
                            )  # Convert the message to a string to capture its content
                            message_writer.writerow(
                                [timestamp, i + 1, message_type, message_content]
                            )

                    time.sleep(1)  # Log data every second

        except IOError as e:
            print(f"IOError while logging flight data: {e}")

    def _get_smoothed_location(self, drone_index: int) -> Tuple[float, float, float]:
        """
        Get the smoothed location by averaging the x, y, and z coordinates of a drone,
        while excluding outliers based on their deviation from the mean standard deviations.

        This function collects multiple samples of the drone's coordinates, calculates the mean
        and standard deviation for the x, y, and z coordinates, and excludes any outliers that
        deviate beyond a set threshold. The final smoothed location is the average of the
        remaining valid samples.

        Parameters
        ----------
        drone_index : int
            The index of the drone in the vehicles list whose location is to be smoothed.

        Returns
        -------
        Tuple[float, float, float]
            The smoothed (x, y, z) coordinates of the selected drone.

        Raises
        ------
        IndexError
            If the drone_index is out of bounds of the vehicles list.
        """
        xs = []
        ys = []
        zs = []
        num_samples = 100
        deviation_threshold = 2.0

        # Collect data
        for _ in range(num_samples):
            x, y, z = self.get_current_state()[drone_index]
            xs.append(x)
            ys.append(y)
            zs.append(z)
            time.sleep(0.1)  # Short delay

        # Compute mean
        x_mean = sum(xs) / num_samples
        y_mean = sum(ys) / num_samples
        z_mean = sum(zs) / num_samples

        # Calculate 2D Euclidean distances from the mean
        distances = []
        for x, y in zip(xs, ys):
            distance = math.sqrt((x - x_mean) ** 2 + (y - y_mean) ** 2)
            distances.append(distance)

        # Calculate standard deviation of distances
        mean_distance = sum(distances) / num_samples
        xy_std = math.sqrt(
            sum((d - mean_distance) ** 2 for d in distances) / (num_samples - 1)
        )
        z_std = math.sqrt(sum((z - z_mean) ** 2 for z in zs) / (num_samples - 1))

        # Calculate valid indices
        valid_indices = [
            (
                distances[i] <= deviation_threshold * xy_std
                and abs(zs[i] - z_mean) <= deviation_threshold * z_std
            )
            for i in range(num_samples)
        ]

        valid_xs = [xs[i] for i in range(num_samples) if valid_indices[i]]
        valid_ys = [ys[i] for i in range(num_samples) if valid_indices[i]]
        valid_zs = [zs[i] for i in range(num_samples) if valid_indices[i]]

        # Compute average of valid data
        avg_x = sum(valid_xs) / len(valid_xs) if valid_xs else x_mean
        avg_y = sum(valid_ys) / len(valid_ys) if valid_ys else y_mean
        avg_z = sum(valid_zs) / len(valid_zs) if valid_zs else z_mean

        return (avg_x, avg_y, avg_z)

    def _get_relative_position(
        self, current_positions: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """
        Calculate and return the relative position of each drone from the origin.

        This method calculates the relative (x, y, z) positions of each drone with respect to
        a predefined origin point. If the environment is not initialized, it returns the
        current positions without modification.

        Parameters
        ----------
        current_positions : List[Tuple[float, float, float]]
            A list of tuples representing the current (x, y, z) coordinates of each drone.

        Returns
        -------
        List[Tuple[float, float, float]]
            A list of tuples where each tuple contains the relative (x, y, z) position of a drone
            from the origin. If the environment is not initialized, returns the absolute positions.

        """
        relative_positions = []

        if self.is_initialized_env == True:
            for i, _ in enumerate(self.vehicles):
                # Get the current absolute coordinates of the drone
                # lat, lon, alt = self._get_drone_coordinates(drone)

                # Convert the absolute coordinates to (x, y, z)
                current_x, current_y, current_z = current_positions[i]

                # Calculate the relative position from the origin
                relative_x = current_x - self.origin[0]
                relative_y = current_y - self.origin[1]
                relative_z = current_z - self.origin[2]

                # Append the relative position to the list
                relative_positions.append((relative_x, relative_y, relative_z))
        else:  ## TODO that check must be done in get_current_state
            for i, _ in enumerate(self.vehicles):
                # Get the current absolute coordinates of the drone
                # lat, lon, alt = self._get_drone_coordinates(drone)

                # Convert the absolute coordinates to (x, y, z)
                current_x, current_y, current_z = current_positions[i]

                # Append the relative position to the list
                relative_positions.append((current_x, current_y, current_z))

        return relative_positions

    def _check_is_armed(self) -> List[bool]:
        """
        Check if all connected drones are armed.

        This method retrieves the latest heartbeat message from each connected drone
        and determines whether each drone is armed based on the MAVLink mode flag. If
        the heartbeat message is not received or an error occurs, the armed status is
        recorded as False.

        Returns
        -------
        List[bool]
            A list of booleans where each entry corresponds to the armed status of a drone.
            True if the drone is armed, False otherwise.

        Raises
        ------
        ValueError
            If there is an issue retrieving the heartbeat message from any drone.
        """
        armed_status = []
        for i, vehicle in enumerate(self.vehicles):
            try:
                # Retrieve the latest heartbeat message
                vehicle.flush()  # Clear any stale messages
                heartbeat = vehicle.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=5
                )
                if heartbeat is None:
                    raise ValueError(f"No HEARTBEAT received for drone {i+1}")

                # Extract the armed status
                is_armed = (
                    heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    != 0
                )

                print(f"Drone {i+1} armed: {is_armed}")
                armed_status.append(is_armed)

            except Exception as e:
                print(f"Failed to check armed status for drone {i+1}: {e}")
                armed_status.append(False)  # Assuming false if there's an error

        return armed_status

    def _arm_drone(self, vehicle_index: int):
        """
        Arms a specific drone and sets its flight mode to "GUIDED".

        This method sends an arm command to the specified drone, waits for an acknowledgment,
        and if the arm command is accepted, it sets the drone's flight mode to "GUIDED".

        Args:
            vehicle_index (int): The index of the drone in the `self.vehicles` list to be armed.

        Raises:
            ValueError: If the drone cannot be armed or if an error occurs during the process.
        """
        try:
            print(f"Arming drone {vehicle_index+1}...")
            vehicle = self.vehicles[vehicle_index]

            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            ack_msg = try_recv_match(vehicle, message_name="COMMAND_ACK")
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self._set_mode(vehicle, "GUIDED")
                print(f"Drone {vehicle_index+1} armed")
            else:
                print(f"Failed to arm drone {vehicle_index+1}")  # TODO assert exception
        except Exception as e:
            raise ValueError(f"Failed to arm drone {vehicle_index+1}: {e}")

    def _arm_drones(self, vehicle_indices: list[int]):
        """
        Arms the drones specified by the provided list of vehicle indices using threads.

        This method creates and starts a separate thread for each drone in the `vehicle_indices` list
        to arm the drones concurrently. It waits for all threads to complete before returning.

        Parameters:
            vehicle_indices (list[int]): List of indices corresponding to the drones that need to be armed.

        Raises:
            ValueError: If an invalid index is provided or if there is an issue creating or starting the thread.
        """
        threads = []

        for i in vehicle_indices:
            if 0 <= i < len(self.vehicles):
                thread = threading.Thread(target=self._arm_drone, args=(i,))
                threads.append(thread)
                thread.start()
            else:
                print(f"Index {i} is out of range. Skipping...")

        for thread in threads:
            thread.join()

    def _takeoff(self, drone: mavutil.mavlink_connection, target_altitude: int):
        """
        Command the drone to take off and wait until it reaches the target altitude.

        This method sends a takeoff command to the specified drone and continuously monitors its altitude.
        It waits until the drone reaches at least 95% of the specified target altitude before returning.

        Args:
            drone (mavutil.mavlink_connection): The drone instance to command.
            target_altitude (float): The altitude to reach in meters. Must be greater than 0.

        Raises:
            ValueError: If the altitude measurement cannot be retrieved or is invalid.
        """
        if target_altitude <= 0:
            return  # Do not proceed with takeoff if altitude is 0 or less

        print("Taking off...")

        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_altitude,
        )

        # Wait until the drone reaches the target altitude
        while True:
            msg = try_recv_match(drone, message_name="GLOBAL_POSITION_INT")
            altitude = msg.relative_alt / 1000.0  # altitude in meters
            print(f"Altitude: {altitude}")
            if altitude >= target_altitude * 0.95:  # 95% of target altitude
                print("Reached target altitude")
                break
            time.sleep(0.5)  # Wait before checking altitude again

    def takeoff(self, target_altitudes: list[int]):
        """
        Initiates takeoff for all drones to their respective target altitudes.
        Spawns a thread for each drone to perform the takeoff concurrently.

        Args:
            target_altitudes (list[int]): List of target altitudes for each drone.

        Raises:
            ValueError: If the number of vehicles does not match the number of target altitudes.
        """

        # Check if the number of vehicles matches the number of target altitudes
        if len(self.vehicles) != len(target_altitudes):
            raise ValueError(
                "The number of vehicles does not match the number of target altitudes."
            )
            return  # Exit the function after raising the exception

        threads = []

        for i, drone in enumerate(self.vehicles):
            try:
                print(f"Taking off drone {i+1} to altitude {target_altitudes[i]}...")
                # Create a thread for each drone takeoff operation
                t = threading.Thread(
                    target=self._takeoff, args=(drone, target_altitudes[i])
                )
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to take off drone {i+1}: {e}")

        # Wait for all threads to complete
        for t in threads:
            t.join()  # Ensure all takeoff operations complete before exiting the function

    def land_drones(self):
        """
        Commands all drones to land simultaneously.
        Spawns a thread for each drone to perform the landing concurrently.
        """
        threads = []

        for i, vehicle in enumerate(self.vehicles):
            try:
                print(f"Landing drone {i+1}...")
                # Create a thread for each drone landing operation
                t = threading.Thread(target=self._land_drone, args=(vehicle,))
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to land drone {i+1}: {e}")

        # Wait for all threads to complete
        for t in threads:
            t.join()  # Ensure all landing operations complete before exiting the function"

    def _land_drone(self, vehicle_index: int) -> None:
        """
        Commands all drones to land simultaneously.
        """

        vehicle = self.vehicles[vehicle_index]
        print(f"Landing drone {vehicle_index+1}...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(0.3)  # TODO check if the waiting time is too long
        while True:
            msg = try_recv_match(vehicle, message_name="GLOBAL_POSITION_INT")
            altitude = msg.relative_alt / 1000.0  # altitude in meters
            print(f"Drone {vehicle_index+1} Altitude:", altitude)
            if altitude <= 0.3:
                print(f"Drone {vehicle_index+1} Landed")
                break
            time.sleep(1)  # TODO check if the waiting time is too long

    def _check_battery(
        self,
    ) -> List[int]:  # TODO state this implementation does not work in docstring
        """
        Checks the battery status of all connected drones.
        """
        battery_status = []
        for i, vehicle in enumerate(self.vehicles):
            try:
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                battery_msg = try_recv_match(vehicle, message_name="BATTERY_STATUS")
                print(f"Drone {i+1} battery status: {battery_msg}")
                battery_status.append(battery_msg)
            except Exception as e:
                raise ValueError(f"Failed to get battery status for drone {i+1}: {e}")

        return battery_status

    def wait_drones(self) -> None:
        """
        Keeps each drone in its current position by switching to LOITER mode if not already in LOITER mode.
        """
        try:
            for vehicle_index, vehicle in enumerate(self.vehicles):
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                    0,
                    0,
                    0,
                    0,
                )
                print(
                    f"Switching drone {vehicle_index+1} to LOITER mode to maintain position."
                )
        except Exception as e:
            pass

    def get_current_state(
        self,
    ) -> List[Tuple[float, float, float]]:
        """
        Get the x, y, z coordinates for each drone.

        Args:
            drones (Dict[int, mavutil.mavlink_connection]): Dictionary of drone connections.

        Returns:
            List[Tuple[float, float, float]]: List of x, y, z coordinates for each drone.
        """
        xyz_coords = []

        for drone in self.vehicles:
            lat, lon, alt = self._get_drone_coordinates(drone)
            x, y, z = lla_to_xyz(lat, lon, alt)
            xyz_coords.append((x, y, z))

        current_relative_coords = self._get_relative_position(xyz_coords)

        return current_relative_coords

    def _get_drone_coordinates(
        self,
        drone: mavutil.mavlink_connection,
    ) -> Tuple[float, float, float]:
        """
        Retrieve the current GPS coordinates of the drone.

        Args:
            drone (mavutil.mavlink_connection): The drone connection.

        Returns:
            Tuple[float, float, float]: The latitude, longitude, and altitude of the drone.
        """
        # Request GPS position
        msg = try_recv_match(drone, message_name="GLOBAL_POSITION_INT")
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = (
            msg.relative_alt / 1000.0
        )  # TODO check if relative_alt is correct for this implementation
        return latitude, longitude, altitude

    def close_environment(self) -> None:
        """
        Closes the connection to all vehicles.
        """
        for i, vehicle in enumerate(self.vehicles):
            print(f"Closing connection to vehicle {i+1}")
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            vehicle.close()
        self.vehicles = []

    def swarm_move(self, offset: Tuple[float, float, float]):
        current_positions = self.get_current_state()
        new_targets = []

        # Calculate new target coordinates for each drone
        for position in current_positions:
            new_x = position[0] + offset[0]
            new_y = position[1] + offset[1]
            new_z = position[2] + offset[2]
            new_targets.append((new_x, new_y, new_z))

        # Move all drones to their new target coordinates simultaneously
        threads = []
        for i, drone in enumerate(self.vehicles):
            try:
                t = threading.Thread(
                    target=self._move, args=(drone, new_targets[i], self.origin)
                )
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to move drone {i+1}: {e}")

        # Ensure all threads have completed
        for t in threads:
            t.join()

    def move(self, coords: List[Tuple[float, float, float]]):
        threads = []
        for i, drone in enumerate(self.vehicles):
            try:
                print(f"Moving drone {i+1} to {coords[i]}...")
                t = threading.Thread(
                    target=self._move, args=(drone, coords[i], self.origin)
                )
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to move drone {i+1}: {e}")

    def _move(
        self,
        drone: mavutil.mavlink_connection,
        target_coordinates: Tuple[float, float, float],
        origin: Tuple[float, float, float],
    ) -> None:
        """
        Move a drone to the target coordinates.

        Args:
            drone (mavutil.mavlink_connection): The drone connection.
            target_coordinates (Tuple[float, float, float]): Target coordinates (x, y, z).
        """
        x, y, z = target_coordinates

        avg_x, avg_y, avg_z = origin[0], origin[1], origin[2]
        target_x, target_y, target_z = (avg_x + x), (avg_y + y), (avg_z + z)
        lat, lon, alt = xyz_to_lla(target_x, target_y, target_z)
        send_position_target_global_int(drone, lat, lon, alt)

    def _force_disarm(self, vehicle: mavutil.mavlink_connection):
        """
        Force disarms the vehicle.
        """
        # Send disarm command directly
        set_parameter(vehicle, "MOT_SAFE_DISARM", 1)
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation (set to 0)
            0,  # Disarm (0 to disarm, 1 to arm)
            21196,  # Magic number to force disarm
            0,
            0,
            0,
            0,
            0,  # Unused parameters
        )

        # Wait for ACK command using try_recv_match
        ack_msg = try_recv_match(vehicle, "COMMAND_ACK", retries=5, timeout=2)

        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Force disarm command accepted")
            else:
                print(f"Force disarm command failed with result: {ack_msg.result}")
        else:
            print("Force disarm command failed or no ACK received")

    def _force_disarm_multiple(self, vehicle_indices: list[int]):
        """
        Force disarms the drones corresponding to the provided list of vehicle indices using threads.

        Parameters:
        vehicle_indices (list[int]): List of indices of drones to force disarm.
        """
        threads = []

        for i in vehicle_indices:
            if 0 <= i < len(self.vehicles):
                vehicle = self.vehicles[i]
                thread = threading.Thread(target=self._force_disarm, args=(vehicle,))
                threads.append(thread)
                thread.start()
            else:
                print(f"Index {i} is out of range. Skipping...")

        for thread in threads:
            thread.join()
