import math

CENTER_X = 320
CENTER_Y = 256
POINT_X = 558
POINT_Y = 328

delta_x = (CENTER_X - POINT_X)**2
delta_y = (CENTER_Y - POINT_Y)**2
distance_in_pixels = (delta_y + delta_x)**0.5
distance = distance_in_pixels * 0.38

angle_radian = math.atan2(delta_y, delta_x)
angle_degrees = angle_radian*(180/math.pi)

print(distance)
print(angle_degrees)





