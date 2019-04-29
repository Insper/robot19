import math


EPS = 1e-8


def dist_sq(p1, p2):
    """Square distance between points p1 and p2"""
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2


def dist(p1, p2):
    """Distance between points p1 and p2"""
    return math.sqrt(dist_sq(p1, p2))


def my_atan2(y, x):
    """atan2 in range [0, 2*pi]."""
    theta = math.atan2(y, x)
    if theta < 0:
        theta += 2 * math.pi
    return theta
