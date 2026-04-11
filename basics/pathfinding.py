import math
import L3_path_template as path

def go_to_position(x, y):
    # Get theta
    if (x >= 0) and (y >= 0):             # Quadrant 1
        theta = -math.atan(x / y)
        rotation_time = 1
    elif (x <= 0) and (y >= 0):           # Quadrant 2
        theta = -math.atan(x / y)
        rotation_time = 1
    elif (x <= 0) and (y <= 0):           # Quadrant 3
        theta = (math.pi) - math.atan(x / y)
        theta = theta / 2       # Halfs speed
        rotation_time = 2       # Doubles Time
    elif (x >= 0) and (y <= 0):           # Quadrant 4
        theta = ((3 * math.pi) / 2) + math.atan(x / y)
        theta = theta / 2       # Halfs Speed
        rotation_time = 2       # Doubles Time
    
    # Get distance
    dist = math.sqrt((x**2) + y**2)
    dist_multiplier = dist // 0.25
    print(dist_multiplier)
    dist = dist / dist_multiplier
    dist_time = dist_multiplier

    print(theta, x)

    path.drive([[0, theta, rotation_time],
                [dist, 0.0725, dist_time]])




if __name__ == "__main__":
    while (1):
        print('howdy')
        x = float(input("x:"))
        y = float(input("y:"))
        go_to_position(x, y)