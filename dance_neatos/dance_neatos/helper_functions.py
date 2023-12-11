import math

def convert(key_x, key_y):
    x_length = 650
    y_length = 500

    x_length_m = 3.25
    y_length_m = 2.5

    location_x = key_x / x_length * x_length_m
    location_y = - key_y / y_length * y_length_m

    return location_x, location_y


# starting location
# find on map

def quaternion_to_euler(quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
        t1 = +1.0 - 2.0 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1])
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        t4 = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z
