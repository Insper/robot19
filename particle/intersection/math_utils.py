import math


EPS = 1e-8
EPS_SQ = EPS * EPS
THETA_EPS = .0002 # Approximately .01 degree


def dist_sq(p1, p2):
    """Square distance between points p1 and p2"""
    x = p1[0] - p2[0]
    y = p1[1] - p2[1]
    return x*x + y*y


def dist(p1, p2):
    """Distance between points p1 and p2"""
    return math.sqrt(dist_sq(p1, p2))


def my_atan2(y, x):
    """atan2 in range [0, 2*pi]."""
    theta = math.atan2(y, x)
    if theta < 0:
        theta += 2 * math.pi
    return theta
