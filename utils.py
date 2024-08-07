from pyproj import Transformer, CRS
from typing import Tuple, List, Dict
import math
import time
from typing import Tuple
from pymavlink import mavutil

wgs84 = CRS("EPSG:4326")  # WGS84
turef30 = CRS("EPSG:5254")  # TUREF30
lla_to_xyz_transformer = Transformer.from_crs(wgs84, turef30, always_xy=True)
xyz_to_lla_transformer = Transformer.from_crs(turef30, wgs84, always_xy=True)


def lla_to_xyz(
    latitude: float, longitude: float, altitude: float
) -> Tuple[float, float, float]:
    """
    Converts latitude, longitude, and altitude to XYZ coordinates.
    Uses WGS84 and TUREF30 coordinate systems for the conversion.

    Parameters
    ----------
        latitude (float):
            Latitude in degrees in WGS84 coordinate system
        longitude (float):
            Longitude in degrees in WGS84 coordinate system
        altitude (float):
            Altitude in meters in WGS84 coordinate system

    Returns
    ----------
        Tuple[float, float, float]:
            Tuple containing the X, Y, and Z coordinates in TUREF30 coordinate system
    """
    x, y, z = lla_to_xyz_transformer.transform(longitude, latitude, altitude)
    return x, y, z


def xyz_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Converts XYZ coordinates to latitude, longitude, and altitude.
    Uses TUREF30 and WGS84 coordinate systems for the conversion.

    Parameters
    ----------
        x (float):
            X coordinate in TUREF30 coordinate system
        y (float):
            Y coordinate in TUREF30 coordinate system
        z (float):
            Z coordinate in TUREF30 coordinate system

    Returns
    ----------
        Tuple[float, float, float]:
            Tuple containing the latitude, longitude, and altitude in WGS84 coordinate system
    """
    longitude, latitude, altitude = xyz_to_lla_transformer.transform(x, y, z)
    return latitude, longitude, altitude


def send_position_target_global_int(
    drone: mavutil.mavlink_connection,
    lat: float,
    lon: float,
    alt: float,
    vx: float = 0,
    vy: float = 0,
    vz: float = 0,
    yaw: float = 0,
    yaw_rate: float = 0,
) -> None:
    """
    Send a position target message to the drone using global coordinates.

    Args:
        drone (mavutil.mavlink_connection): The drone connection.
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        alt (float): Altitude in meters.
        vx (float): X velocity in m/s.
        vy (float): Y velocity in m/s.
        vz (float): Z velocity in m/s.
        yaw (float): Yaw angle in radians.
        yaw_rate (float): Yaw rate in radians/second.
    """
    drone.mav.send(
        drone.mav.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            int(lat * 1e7),
            int(lon * 1e7),
            alt,  # lat, lon, alt
            vx,
            vy,
            vz,  # x, y, z velocity in m/s (not used)
            0,
            0,
            0,  # afx, afy, afz acceleration (not used)
            yaw,
            yaw_rate,  # yaw, yaw_rate (not used)
        )
    )


def calculate_average_coordinates(
    xyz_coords: List[Tuple[float, float, float]]
) -> Tuple[float, float, float]:
    """
    Calculate the average of the x, y, z coordinates.

    Args:
        xyz_coords (List[Tuple[float, float, float]]): List of x, y, z coordinates.

    Returns:
        Tuple[float, float, float]: The average x, y, z coordinates.
    """
    avg_x = sum(coord[0] for coord in xyz_coords) / len(xyz_coords)
    avg_y = sum(coord[1] for coord in xyz_coords) / len(xyz_coords)
    avg_z = sum(coord[2] for coord in xyz_coords) / len(xyz_coords)
    return (avg_x, avg_y, avg_z)


def set_parameter(drone, param_id, param_value):
    """
    Set a parameter on the drone.

    Args:
        drone: MAVLink connection object.
        param_id: The parameter id (name) as string.
        param_value: The parameter value to set.

    Returns:
        None
    """
    drone.mav.param_set_send(
        drone.target_system,
        drone.target_component,
        param_id.encode("utf-8"),  # Encode the parameter ID to bytes
        float(param_value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Wait for the parameter to be set
    while True:
        ack = drone.recv_match(type="PARAM_VALUE", blocking=True)
        if ack and ack.param_id == param_id:
            print(f"Parameter {param_id} set to {ack.param_value}")
            break


def get_attitude(drone):
    """
    Get the attitude (roll, pitch, yaw) of the drone.

    Args:
        drone (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        tuple: Roll, pitch, and yaw angles in degrees.
    """
    msg = drone.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = msg.roll * 180.0 / 3.14159265
        pitch = msg.pitch * 180.0 / 3.14159265
        yaw = msg.yaw * 180.0 / 3.14159265
        return roll, pitch, yaw
    return None, None, None


def display_real_time_attitude(drone):
    """
    Display the real-time roll, pitch, and yaw angles of the drone.

    Args:
        drone (mavutil.mavlink_connection): The MAVLink connection object.
    """
    print("Displaying real-time roll, pitch, and yaw angles...")
    try:
        while True:
            roll, pitch, yaw = get_attitude(drone)
            if roll is not None:
                print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting real-time attitude display.")
        
        
# def main():
#     xyz_cords, avg_x, avg_y, avg_z = connect_drones()
    
#     for drone in drones:
#         display_real_time_attitude(drone)