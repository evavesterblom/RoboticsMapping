import numpy as np
import math

def parametric_equation(start: tuple, end: tuple, count=100):
    """
    Returns a list of points on the line between start and end.
    """
    t_values = np.linspace(0, 1, count)
    points = np.array([np.array(start) + t * (np.array(end) - np.array(start)) for t in t_values])
    return points

def dda(start, end, wall_size):
    x1, y1 = start
    x2, y2 = end
    points = []
    dx = x2 - x1
    dy = y2 - y1
    steps = math.ceil(max(abs(dx), abs(dy)) / wall_size)
    x_inc = dx / wall_size / steps
    y_inc = dy / wall_size / steps
    x1 /= wall_size
    y1 /= wall_size
    for i in range(0, steps):
        x1 = x1 + x_inc
        y1 = y1 + y_inc
        points.append((int(x1), int(y1)))
    return points

