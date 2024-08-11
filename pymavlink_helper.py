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


class PyMavlinkHelper:
    """
    PyMavlink environment helper class that provides a high-level interface to interact with the environment.
    """

    def __init__(
        self,
        drone_count: int,
        connection_strings: List[str],
        baud_rate: int,
    ) -> None:
        """
        Initializes the PyMavlinkHelper class.

        Parameters
        ----------
        drone_count (int):
            The number of drones to be managed.
        connection_strings (List[str]):
            List of connection strings for each drone.
        baud_rate (int):
            Baud rate for the connection.
        """
        self.drone_count = drone_count
        self.connection_strings = connection_strings
        self.baud_rate = baud_rate
        self.vehicles = []

    def _set_mode(self, drone: mavutil.mavlink_connection, mode: str) -> None:
        """
        Set the flight mode of the drone.

        Args:
            drone (mavutil.mavlink_connection): The drone connection.
            mode (str): The flight mode to set (e.g., "GUIDED", "LOITER", "RTL").
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
        Connect to the vehicles using the provided connection strings and baud rate.
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
            coords = self.get_smoothed_location(drone_index)
            initial_coords.append(coords)

        self.origin = calculate_average_coordinates(initial_coords)
        self.is_initialized_env = True
        self.logging_thread = threading.Thread(target=self._log_flight)
        self.logging_thread.start()

        print("All vehicles connected.")
        print(f"Origin is {self.origin}")

    def _log_flight(self):
        """
        Logs the flight data of the drones to a CSV file.
        """
        with open(self.log_coords_file, mode="a", newline="") as coords_file, open(
            self.message_log_file, mode="a", newline=""
        ) as message_file:
            coord_writer = csv.writer(coords_file)
            message_writer = csv.writer(message_file)
            coord_writer.writerow(
                ["Timestamp", "DroneID", "Latitude", "Longitude", "Altitude"]
            )
            message_writer.writerow(["Timestamp", "DroneID", "Message"])

            while self.is_initialized_env:
                drone_coords = self.get_current_state()
                for i, drone in enumerate(self.vehicles):
                    # Log coordinates
                    x, y, z = drone_coords[i]
                    x, y, z = round(x, 2), round(y, 2), round(z, 2)
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    coord_writer.writerow([timestamp, i + 1, x, y, z])

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

    def get_smoothed_location(self, drone_index: int) -> Tuple[float, float, float]:
        """
        Get the smoothed location by averaging x, y, and z coordinates,
        excluding outliers based on deviation from the mean standard deviations.

        Args:
            drone_index (int): Index of the drone in the drones list.

        Returns:
            Tuple[float, float, float]: Smoothed x, y, and z coordinates of the selected drone.
        """
        xs = []
        ys = []
        zs = []
        num_samples = 10
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

        Returns:
            List[Tuple[float, float, float]]: A list of relative (x, y, z) positions for each drone.
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
        else:
            for i, _ in enumerate(self.vehicles):
                # Get the current absolute coordinates of the drone
                # lat, lon, alt = self._get_drone_coordinates(drone)

                # Convert the absolute coordinates to (x, y, z)
                current_x, current_y, current_z = current_positions[i]

                # Append the relative position to the list
                relative_positions.append((current_x, current_y, current_z))

        return relative_positions

    ## TODO this function trying arm and if it is succeded return true. it is not actually checking.
    def _check_is_armed(self) -> List[bool]:
        """
        Checks if all the connected drones are armed.
        TODO add Raises part to the function
        """
        armed_status = []
        for i, vehicle in enumerate(self.vehicles):
            try:
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
                ack_msg = try_recv_match(vehicle, message_name="COMMAND_ACK")
                is_armed = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                print(f"Drone {i+1} armed: {is_armed}")  # TODO
                armed_status.append(is_armed)
            except Exception as e:
                raise ValueError(f"Failed to check armed status for drone {i+1}: {e}")

        return armed_status

    def _arm_drone(self, vehicle_index: int):
        """
        Arms all the connected drones.
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
        Arms the drones corresponding to the provided list of vehicle indices using threads.

        Parameters:
        vehicle_indices (list[int]): List of indices of drones to arm.
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

    def _takeoff(self, drone: mavutil.mavlink_connection, target_altitude):
        """
        Command the drone to take off and wait until it reaches the target altitude.

        Args:
        drone: The drone instance to command.
        target_altitude (float): The altitude to reach in meters.
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

    # def _get_absolute_coords(
    #     self,
    # ) -> List[Tuple[float, float, float]]:
    #     """
    #     Get the x, y, z coordinates for each drone.

    #     Args:
    #         drones (Dict[int, mavutil.mavlink_connection]): Dictionary of drone connections.

    #     Returns:
    #         List[Tuple[float, float, float]]: List of x, y, z coordinates for each drone.
    #     """
    #     absolute_coords = []

    #     for drone in self.vehicles:
    #         lat, lon, alt = self._get_drone_coordinates(drone)
    #         absolute_coords.append((lat, lon, alt))

    #     return absolute_coords

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
