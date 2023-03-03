def world_to_grid(x, y, origin_x, origin_y, width, height, resolution):
    # grid where x,y falls
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)

    # check boundaries
    x_boundary = int(width / resolution)
    y_boundary = int(height / resolution)
    if grid_x <= x_boundary and grid_y <= y_boundary and x >= origin_x and y >= origin_y:
        return grid_x, grid_y
    else:
        return None


def grid_to_world(gx, gy, origin_x, origin_y, width, height, resolution):
    return gx * resolution + origin_x + (resolution / 2) * pow(-1, gx < 0),\
        gy * resolution + origin_y + (resolution / 2) * pow(-1, gy < 0)


# TODO!!!!! NOT SURE IT IS CORRECT ATM!!!!!!
def grid_to_world_2(gx, gy, origin_x, origin_y, width, height, resolution):
    return None


def print_world_info(origin_x, origin_y, width, height, resolution):
    print(
        f"Grid size: {int(width / resolution)}x{int(height / resolution)} cells, each {resolution}x{resolution} meters")
    print(f"Grid origin: ({origin_x}, {origin_y})")
    print(f"Grid boundaries: ({origin_x}, {origin_y}) - ({origin_x + width}, {origin_y + height})")


def test_grid_to_world(origin_x=0, origin_y=0, width=100, height=100, resolution=10):
    print_world_info(origin_x, origin_y, width, height, resolution)
    # Test the function with some example points
    points = [(1, 1), (5, 7), (-5, 0), (10, 15), (0, 0)]
    for point in points:
        world_cords = grid_to_world(point[0], point[1], origin_x, origin_y, width, height, resolution)
        print(f"Point {point} is in world coords {world_cords}")

    print(f"Input: Start {(origin_x, origin_y)}, Max {(origin_x + width, origin_y + height)}, Resolution {resolution}")


def test_world_to_grid(origin_x=0, origin_y=0, width=100, height=100, resolution=10):
    print_world_info(origin_x, origin_y, width, height, resolution)
    # Test the function with some example points
    points = [(10, 10), (55, 70), (-5, 20), (120, 80), (0, 0)]
    for point in points:
        grid_coords = world_to_grid(point[0], point[1], origin_x, origin_y, width, height, resolution)
        if grid_coords:
            print(f"World to Grid: Point {point} is in grid cell {grid_coords}")
        else:
            print(f"World to Grid: Point {point} is outside the grid")


if __name__ == '__main__':
    test_world_to_grid()
    test_grid_to_world()
