from pyproj import Transformer, CRS
from typing import Tuple, List, Dict
import math
import time
from typing import Tuple
import mavlink
from pymavlink import mavutil

wgs84 = CRS("EPSG:4326")  # WGS84
turef30 = CRS("EPSG:5254")  # TUREF30
lla_to_xyz_transformer = Transformer.from_crs(wgs84, turef30, always_xy=True)
xyz_to_lla_transformer = Transformer.from_crs(turef30, wgs84, always_xy=True)


def lla_to_xyz(
    self, latitude: float, longitude: float, altitude: float
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


def xyz_to_lla(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
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
    return avg_x, avg_y, avg_z