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
        log_coords_file: str = "flight_coord_log.csv",
        message_log_file: str = "message_log.csv",
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
        log_coords_file (str):
            The CSV file to log the flight data.
        """
        self.drone_count = drone_count
        self.connection_strings = connection_strings
        self.baud_rate = baud_rate
        self.log_coords_file = log_coords_file
        self.message_log_file = message_log_file
        self.vehicles = []
        self.is_initialized_env = False

    def _ack_msg(self, drone: mavutil.mavlink_connection, command: int) -> str:
        """
        Waits for and processes the ACK message for a given command.

        Args:
            drone (mavutil.mavlink_connection): The drone connection.
            command (int): The MAVLink command to wait for ACK.

        Returns:
            str: The result of the command ('accepted', 'denied', 'in progress', 'failed', etc.).

        Raises:
            RuntimeError: If an error occurs while processing the ACK message.
        """
        try:
            ack = None
            while not ack:
                ack = drone.recv_match(type="COMMAND_ACK", blocking=True)
                if ack and ack.command == command:
                    result = ack.result
                    if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        return "accepted"
                    elif result == mavutil.mavlink.MAV_RESULT_DENIED:
                        return "denied"
                    elif result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                        return "in progress"
                    elif result == mavutil.mavlink.MAV_RESULT_FAILED:
                        return "failed"
                    else:
                        return f"unknown result ({result})"
        except AttributeError as e:
            raise RuntimeError(f"Error processing ACK message: {e}")
        except Exception as e:
            raise RuntimeError(f"Failed to receive ACK message: {e}")
        return "no ack received"

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
        # Wait for ACK command
        if self._ack_msg(drone, mavutil.mavlink.MAV_CMD_DO_SET_MODE):
            print(f"Mode change to {mode} accepted")
        else:
            print(f"Mode change to {mode} failed")  # TODO assert exception here

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

    def _check_is_armed(self) -> List[bool]:
        """
        Checks if all the connected drones are armed.

        Returns:
            List[bool]: A list of booleans indicating the armed status of each drone.

        Raises:
            ValueError: If there is an error while checking the armed status for any drone.
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
                ack_msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
                is_armed = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                print(f"Drone {i+1} armed: {is_armed}")
                armed_status.append(is_armed)
            except Exception as e:
                raise ValueError(f"Failed to check armed status for drone {i+1}: {e}")

        return armed_status

    def _arm_drones(self):
        """
        Arms all the connected drones.
        """
        for i, vehicle in enumerate(self.vehicles):
            try:
                print(f"Arming drone {i+1}...")
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
                # Wait for ACK command
                if self._ack_msg(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                    self._set_mode(vehicle, "GUIDED")
                    print(f"Drone {i+1} armed")
                else:
                    print(f"Failed to arm drone {i+1}")
            except Exception as e:
                raise ValueError(f"Failed to arm drone {i+1}: {e}")

    def _takeoff(self, drone, target_altitude):
        """
        Command the drone to take off and wait until it reaches the target altitude.

        Args:
        drone: The drone instance to command.
        target_altitude (float): The altitude to reach in meters.
        """
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

        # Wait for ACK command
        if self._ack_msg(drone, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
            print("Takeoff command accepted")
        else:
            print("Takeoff command failed")

        # Wait until the drone reaches the target altitude
        while True:
            msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            altitude = msg.relative_alt / 1000.0  # altitude in meters
            print(f"Current altitude: {altitude}")
            if altitude >= target_altitude * 0.95:  # 95% of target altitude
                print("Reached target altitude")
                break
            time.sleep(0.5)

    def takeoff(self, target_altitude):
        threads = []
        for i, drone in enumerate(self.vehicles):
            try:
                print(f"Taking off drone {i+1} to altitude {target_altitude}...")
                t = threading.Thread(
                    target=self._takeoff, args=(drone, target_altitude)
                )
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to take off drone {i+1}: {e}")

        for t in threads:
            t.join()

    def land_drones(self, velocity: float, descent_rate: float) -> None:
        """
        Commands all drones to land simultaneously with specified velocity and descent rate.

        Parameters:
        velocity (float): The velocity to set for landing (m/s).
        descent_rate (float): The descent rate for landing (m/s^2).
        """
        for i, vehicle in enumerate(self.vehicles):
            print(f"Setting landing speed for drone {i+1}...")
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1,  # Airspeed
                velocity,
                0,
                0,
                0,
                0,
                0,
            )

            print(f"Setting descent rate for drone {i+1}...")
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
                0,
                descent_rate,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            print(f"Landing drone {i+1}...")
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

            # Wait for ACK command
            if self._ack_msg(vehicle, mavutil.mavlink.MAV_CMD_NAV_LAND):
                print(f"Landing command for drone {i+1} accepted")
            else:
                print(f"Landing command for drone {i+1} failed")

        print("All drones commanded to land.")
        while True:
            all_landed = True
            for vehicle in self.vehicles:
                msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
                altitude = msg.relative_alt / 1000.0  # altitude in meters
                if altitude > 0.1:
                    all_landed = False
            if all_landed:
                print("All drones landed")
                break
            time.sleep(0.5)

    def _check_battery(self) -> List[int]:
        """
        Checks the battery status of all connected drones.

        Note:
            This implementation does not work as expected. The battery status message
            might not be received or parsed correctly.

        Returns:
            List[int]: A list of battery status messages for each drone.

        Raises:
            ValueError: If there is an error while checking the battery status for any drone.
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
                battery_msg = vehicle.recv_match(type="BATTERY_STATUS", blocking=True)
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
        msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = (
            msg.relative_alt / 1000.0
        )  # TODO check if relative_alt is correct for this implementation
        return latitude, longitude, altitude

    def _force_disarm(self, vehicle):
        """
        Force disarms the vehicle.
        """
        # Send disarm command directly
        set_parameter(vehicle, "NOT_SAFE_DISARM", 1)
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

        # Wait for ACK command
        if self._ack_msg(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
            print("Force disarm command accepted")
        else:
            print("Force disarm command failed")

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
            # Wait for ACK command
            if self._ack_msg(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                if self._check_is_armed()[i] != False:
                    print(f"Drone {i+1} disarmed")
                else:
                    self._force_disarm(vehicle)
            else:
                print(f"Failed to disarm drone {i+1}")

            vehicle.close()

        self.vehicles = []
        self.is_initialized_env = False

    def move(self, coords: List[Tuple[float, float, float]]):
        threads = []
        for i, drone in enumerate(self.vehicles):
            try:
                print(f"Moving drone {i+1} to {coords[i]}...")
                t = threading.Thread(target=self._move, args=(drone, coords[i]))
                threads.append(t)
                t.start()
            except Exception as e:
                print(f"Failed to move drone {i+1}: {e}")

        for t in threads:
            t.join()

    def _move(
        self,
        drone: mavutil.mavlink_connection,
        target_coordinates: Tuple[float, float, float],
    ) -> None:
        """
        Move a drone to the target coordinates.

        Args:
            drone (mavutil.mavlink_connection): The drone connection.
            target_coordinates (Tuple[float, float, float]): Target coordinates (x, y, z).
        """
        x, y, z = target_coordinates

        avg_x, avg_y, avg_z = self.origin[0], self.origin[1], self.origin[2]
        target_x, target_y, target_z = (avg_x + x), (avg_y + y), (avg_z + z)
        lat, lon, alt = xyz_to_lla(target_x, target_y, target_z)
        send_position_target_global_int(drone, lat, lon, alt)
