import math


def convert(key_x: float, key_y: float) -> tuple:
    """
    Convert pixel keypoiny coordinates to corresponding physical coordinates.

    Take in a keypoint coordinate and convert that to the location on
    the shared "map" frame. The map is 3.25 x 2.5 m. Note that the webcam's
    Y axis increases when going down, and the map frame's y increases when
    going up.

    Args:
        key_x (float): x-coordinate in pixels.
        key_y (float): y-coordinate in pixels.

    Returns:
        tuple: A tuple containing the converted x and y coordinates in meters.
    """
    # Dimensions of the Webcam field in pixels
    x_length = 650
    y_length = 500

    # Corresponding dimensions in meters
    x_length_m = 3.25
    y_length_m = 2.5

    # Convert x-coordinate to meters
    location_x = key_x / x_length * x_length_m

    # Convert y-coordinate to meters
    if key_y == 0:  # if keypoint is at 0, keep it at 0
        location_y = 0
    else:
        location_y = (y_length - key_y) / y_length * y_length_m

    return location_x, location_y


def quaternion_to_euler(quaternion: tuple) -> tuple:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        quaternion (tuple): A 4-element list or tuple representing the quaternion (w, x, y, z).

    Returns:
        tuple: A tuple containing the Euler angles (roll, pitch, yaw) in radians.
    """
    # Extract quaternion components
    w, x, y, z = quaternion

    # Calculate roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # Calculate pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # Calculate yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z
