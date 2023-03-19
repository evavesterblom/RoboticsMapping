import rospy


def world_to_grid(x, y, origin_x, origin_y, width, height, resolution):
    # grid where x,y falls
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)

    # check boundaries
    x_boundary = origin_x + int(width / resolution)
    y_boundary = origin_y + int(height / resolution)
    if grid_x <= width / resolution and grid_y <= height / resolution and x_boundary >= x >= origin_x and y_boundary >= y >= origin_y:
        return grid_x, grid_y
    else:
        return None

def offset_points(points, offset, grid_size):
    result = []
    for point in points:
        result.append(offset_point(point, offset, grid_size))
    return result

def offset_point(point, offset, grid_size):
    if point[0] + offset[0] < grid_size[0] and point[1] + offset[1] < grid_size[1]:
        return point[0] - offset[0], point[1] - offset[1]
    else:
        raise Exception("Point out of bounds")

def grid_to_world(gx, gy, origin_x, origin_y, width, height, resolution):
    x = gx * resolution + origin_x
    y = gy * resolution + origin_y
    offset = resolution / 2
    return x + offset, y + offset
