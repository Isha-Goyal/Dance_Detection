def convert(key_x, key_y):
    x_length = 650
    y_length = 500

    x_length_m = 3.25
    y_length_m = 2.5

    location_x = key_x / x_length * x_length_m
    location_y = - key_y / y_length * y_length_m

    return location_x, location_y