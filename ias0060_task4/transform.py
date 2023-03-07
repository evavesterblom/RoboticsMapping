import numpy as np 

def world_to_grid(x, y, origin_x, origin_y, width, height, resolution):
    # grid where x,y falls
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)

    # check boundaries
    x_boundary = origin_x + int(width / resolution)
    y_boundary = origin_y + int(height / resolution)
    if (grid_x <= x_boundary and grid_y <= y_boundary and x >= origin_x and y >= origin_y):
        return (grid_x, grid_y)
    else:
        return None
    
def world_to_grid_2(x, y, origin_x, origin_y, width, height, resolution):
    # Compute the grid cell coordinates
    if x > (width + origin_x) or y > (height + origin_y) or x < origin_x or y < origin_y:
        return None
    else:
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
    return grid_x, grid_y
    
def grid_to_world(gx, gy, origin_x, origin_y, width, height, resolution):
    x = gx * resolution + origin_x
    y = gy * resolution + origin_y
    offset = resolution/2
    return (x + offset, y + offset)

def print_world_info(origin_x, origin_y, width, height, resolution):
    print(f"Grid size: {int(width / resolution)}x{int(height / resolution)} cells, each {resolution}x{resolution} meters")
    print(f"Grid origin: ({origin_x}, {origin_y})")
    print(f"Grid boundaries: ({origin_x}, {origin_y}) - ({origin_x + width}, {origin_y + height})")

def test_grid_to_world(origin_x, origin_y, width, height, resolution):
    points = [(1, 1), (5, 7), (-5, 0), (10, 15), (10, 10)]
    for point in points:
        world_cords = grid_to_world(point[0], point[1], origin_x, origin_y, width, height, resolution)
        print(f"Point {point} is in world coords {world_cords}")

def test_world_to_grid(origin_x, origin_y, width, height, resolution):
    points = [(-0.9, 1.0),(10.8, 10.0), (55.0, 70.0), (-5.0, 20.6), (120.0, 80.8)]
    for point in points:
        grid_coords = world_to_grid(point[0], point[1], origin_x, origin_y, width, height, resolution)
        if grid_coords:
            print(f"World to Grid: Point {point} is in grid cell {grid_coords}")
        else:
            print(f"World to Grid: Point {point} is outside the grid")

def test_assert(origin_x, origin_y, width, height, resolution):
    for x in np.arange(-10.5, 100.5, 0.5):
        for y in np.arange(-10.5, 100.5, 0.5):
             #print('%s * %s' % (x, y))
             e = world_to_grid(x, y, origin_x, origin_y, width, height, resolution)
             d = world_to_grid_2(x, y, origin_x, origin_y, width, height, resolution)
             print('XY = (%s, %s) -> %s * %s' % (x, y, e, d))
             assert(e == d, (x, y, e, d)) 


if __name__ == '__main__':
    origin_x = -1
    origin_y = -1
    width = 100 
    height = 100
    resolution = 10
    print_world_info(origin_x, origin_y, width, height, resolution)
    print(f" - - - ")
    test_world_to_grid(origin_x, origin_y, width, height, resolution) 
    print(f" - - - ")     
    test_grid_to_world(origin_x, origin_y, width, height, resolution)
    print(f" - - -  - - - - - - ")  
    test_assert(origin_x, origin_y, width, height, resolution)


    