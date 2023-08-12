from math import asin, atan2, cos, degrees, radians, sin, pi

CENTER_X = 320
CENTER_Y = 256
POINT_X = 558
POINT_Y = 328
AZ = 335
POINT_LAT = 50.603694
POINT_LON = 30.650625

def get_distance_and_angle(x1, y1, x2, y2):
    delta_x = (x1 - x2)**2
    delta_y = (y1 - y2)**2
    distance_in_pixels = (delta_y + delta_x)**0.5
    distance = distance_in_pixels * 0.38

    angle_radian = atan2(delta_y, delta_x)
    angle_degrees = angle_radian*(180/pi)
    return (distance, angle_degrees)

def get_point_at_distance(lat1, lon1, d, bearing, R=6371):
    """
    lat: initial latitude, in degrees
    lon: initial longitude, in degrees
    d: target distance from initial
    bearing: (true) heading in degrees
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    a = radians(bearing)
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1) * sin(d/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(d/R) * cos(lat1),
        cos(d/R) - sin(lat1) * sin(lat2)
    )
    return (degrees(lat2), degrees(lon2),)

distance, angle_degrees = get_distance_and_angle(POINT_X, POINT_Y, CENTER_X, CENTER_Y)

print(get_point_at_distance(POINT_LAT, POINT_LON, distance, angle_degrees))



