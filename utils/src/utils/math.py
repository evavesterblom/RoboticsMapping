import numpy as np


def parametric_equation(start: tuple, end: tuple, count=100):
    """
    Returns a list of points on the line between start and end.
    """
    t_values = np.linspace(0, 1, count)
    points = np.array([np.array(start) + t * (np.array(end) - np.array(start)) for t in t_values])
    return points
