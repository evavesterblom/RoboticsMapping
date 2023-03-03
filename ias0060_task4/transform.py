def world_to_grid(x,y,origin_x,origin_y,width,height,resolution):
    #grid where x,y falls
    grid_x = int((x - origin_x )/ resolution)
    grid_y = int((y - origin_y) / resolution)

    #check boundaries
    x_boundary = width/resolution
    y_boundary = height/resolution
    if (grid_x <= x_boundary and grid_y <= y_boundary and x >= origin_x and y >= origin_y):
        return (grid_x, grid_y) #on see oige?
    else:
        return None



def grid_to_world(gx,gy,origin_x,origin_y,width,height,resolution): #TODO!!!!! NOT SURE IT IS CORRECT ATM!!!!!!
    x_boundary = origin_x + width
    y_boundary = origin_y + height
    if (gx == x_boundary and gy == y_boundary):
        return (x_boundary, y_boundary)
    elif (gx > origin_x and gy > origin_y and gx <= x_boundary and gy <= y_boundary):
        #here if point is out of grid, then return last grid coordinate
        x_center = gx + (resolution / 2)
        y_center = gy + (resolution / 2)
        if (x_center > x_boundary):
            x_center = x_boundary
        if (y_center > y_boundary):
            y_center = y_boundary
        return (y_center, y_center)
    
    else:
        return None

if __name__ == '__main__':
    origin_x = 0
    origin_y = 0
    width = 100 #10 kuubikut
    height = 100
    resolution = 10 

    print(f"Input: Start {(origin_x, origin_y)}, Max {(origin_x + width, origin_y + height)}, Resolution {resolution}")

    # Test the function with some example points
    points = [(10, 10), (55, 70), (-5, 20), (120, 80)]
    for point in points:
        grid_coords = world_to_grid(point[0], point[1], origin_x, origin_y, width, height, resolution)
        world_coords = grid_to_world(point[0], point[1], origin_x, origin_y, width, height, resolution)
        if grid_coords:
            print(f"World to Grid: Point {point} is in grid cell {grid_coords}")
        else:
            print(f"World to Grid: Point {point} is outside the grid")
        if world_coords:
            print(f"Grid to World: Point {point} is in grid cell {world_coords}")
        else:
            print(f"Grid to World: Point {point} is outside the grid")

    