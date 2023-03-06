# Convert world coordinates to grid cell coordinates
def world_to_grid(x, y, origin_x, origin_y, width, height, resolution):
    # Compute the grid cell coordinates
    if x > (width + origin_x) or y > (height + origin_y) or x < origin_x or y < origin_y:
        grid_x = None
        grid_y = None
    else:
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
    return grid_x, grid_y


# Convert grid cell coordinates to the world coordinates
def grid_to_world(gx, gy, origin_x, origin_y, width, height, resolution):
    world_x = (gx + 0.5) * resolution + origin_x
    world_y = (gy + 0.5) * resolution + origin_y
    if world_x > (width + origin_x) or world_y > (height + origin_y) or world_x < origin_x or world_y < origin_y:
        world_x = None
        world_y = None
    return world_x, world_y


# Asking parameters in the terminal and calculate the grid cell coordinates
def print_world_to_grid():
    # Ask the parameters
    origin_x = float(input("Enter the x-coordinate of the map origin: "))
    origin_y = float(input("Enter the y-coordinate of the map origin: "))
    width = float(input("Enter the width of the map: "))
    height = float(input("Enter the height of the map: "))
    resolution = float(input("Enter the map resolution: "))
    x = float(input("Enter x coordinate of a point on the world map: "))
    y = float(input("Enter y coordinate of a point on the world map: "))
    grid_x, grid_y = world_to_grid(x, y, origin_x, origin_y, width, height, resolution)
    return print("The grid cell of the world coordinates are ({}, {}).".format(grid_x, grid_y))


# Asking parameters in the terminal and calculate the world coordinates
def print_grid_to_world():
    # Ask the parameters
    origin_x = float(input("Enter the x-coordinate of the map origin: "))
    origin_y = float(input("Enter the y-coordinate of the map origin: "))
    width = float(input("Enter the width of the map: "))
    height = float(input("Enter the height of the map: "))
    resolution = float(input("Enter the map resolution: "))
    gx = float(input("Enter x coordinate of a map grid cell: "))
    gy = float(input("Enter y coordinate of a map grid cell: "))
    world_x, world_y = grid_to_world(gx, gy, origin_x, origin_y, width, height, resolution)
    return print("The grid cell of the world coordinates are ({}, {}).".format(world_x, world_y))


if __name__ == '__main__':
    print_grid_to_world()
    print_world_to_grid()
