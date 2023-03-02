def world_to_grid(x,y,origin_x,origin_y,width,height,resolution):
    #grid where x,y falls
    grid_x = int((x - origin_x )/ resolution)
    grid_y = int((y - origin_y) / resolution)
    print(grid_x, grid_y)

    #check boundaries
    x_boundary = width/resolution
    y_boundary = height/resolution
    if (grid_x <= x_boundary and grid_y <= y_boundary and x >= origin_x and y >= origin_y):
        return (grid_x, grid_y) #on see oige?
    else:
        return None



def grid_to_world(gx,gy,origin_x,origin_y,width,height,resolution):
    return None

if __name__ == '__main__':
    origin_x = 0
    origin_y = 0
    width = 100 #10 kuubikut
    height = 100
    resolution = 10 

    # Test the function with some example points
    points = [(10, 10), (55, 70), (-5, 20), (120, 80)]
    for point in points:
        grid_coords = world_to_grid(point[0], point[1], origin_x, origin_y, width, height, resolution)
        if grid_coords:
            print(f"Point {point} is in grid cell {grid_coords}")
        else:
            print(f"Point {point} is outside the grid")

    