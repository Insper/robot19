import math
import bisect
import attr
import numpy as np

from .math_utils import EPS, EPS_SQ, THETA_EPS, dist_sq, my_atan2
from .segment import Segment


def create_segments(segment, ref=None):
    """Creates Segment list breaking the segment if direction [1, 0] intersects
    it. This function is used to avoid the case in which the points are sorted
    counterclockwise with reference to ref and theta1 > theta2.
    """
    rx, ry = 0, 0
    if ref is None:
        ref = [0, 0]
    else:
        rx, ry = ref

    p1 = segment.p1
    p2 = segment.p2
    x1 = p1[0] - rx
    y1 = p1[1] - ry
    x2 = p2[0] - rx
    y2 = p2[1] - ry

    if x1 * y2 - x2 * y1 < 0:
        p1, p2 = p2, p1
        x1, y1, x2, y2 = x2, y2, x1, y1

    theta1 = my_atan2(y1, x1)
    theta2 = my_atan2(y2, x2)

    cx = _crossing_x(x1, y1, x2, y2)
    if cx > 0:
        intersection = [cx + rx, ry]
        if abs(theta2) < EPS and abs(theta1) > EPS:
            return [Segment(p1, p2, theta1, 2 * math.pi, ref)]
        elif abs(theta1) < EPS and abs(theta2) > EPS:
            return [Segment(p1, p2, 0, theta2, ref)]
        return [
            Segment(intersection, p2, 0, theta2, ref),
            Segment(p1, intersection, theta1, 2 * math.pi, ref),
        ]
    if abs(theta2) < EPS and theta2 < theta1:
        theta2 = 2 * math.pi
    return [Segment(p1, p2, theta1, theta2, ref)]


def _crossing_x(x1, y1, x2, y2):
    '''
    Verifies if segment defined by (x1, y1) and (x2, y2) crosses the positive
    x axis. If so, returns the value for x where the segment crosses the x
    axis. Otherwise returns a negative number.
    '''
    if (y1 > 0 and y2 > 0) or (y1 < 0 and y2 < 0) or (x1 < 0 and x2 < 0) \
        or abs(y1/x1) <= EPS or abs(y2/x2) <= EPS:
        return -float('inf')
    if y2 > 0:
        x1, y1, x2, y2 = x2, y2, x1, y1
    return (y1 * x2 - y2 * x1) / (y1 - y2)


def make_pts(segment, directs, angles):
    pts = []
    for ang, direct in zip(angles, directs):
        if ang == segment.theta1:
            pts.append(segment.p1)
        elif ang == segment.theta2:
            pts.append(segment.p2)
        else:
            pts.append(segment.intersect(segment.ref, direct, angle=ang)[1])
    return pts


def make_segment(p1, p2, ref=None):
    """Auxiliary function that either creates a Segment or returns None if
    arguments are not valid.
    """
    if p1 is not None and p2 is not None:
        return Segment(p1, p2, ref=ref)
    return None


def create_non_intersecting_segments(pts1, pts2, segs2, angles, ref):
    """Creates list with the Segments that are visible from ref for the case
    when the line segment in pts1 is in front of the line segment in pts2.

    Args:
        pts1 (list[list]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            Segment closest to ref
        pts2 (list[list]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            Segment farthest to ref
        segs2 (list[Segment]): list of Segment between the points in pts2
        angles (list[float]): list of directions (radians) of the rays from ref
            the angles are obtained from the two extremes of each original
            Segment. The list always has 4 elements (even if repeated)
        ref (list): reference point
    """
    segments = []
    segment = Segment(pts1[1], pts1[2], angles[1], angles[2], ref)
    if pts1[0] is not None:
        segment.merge(Segment(pts1[0], pts1[1], angles[0], angles[1], ref))
    if pts1[3] is not None:
        segment.merge(Segment(pts1[2], pts1[3], angles[2], angles[3], ref))
    segments.append(segment)
    if segs2[0] is not None and segs2[0].length_sq > EPS_SQ:
        segments = [segs2[0]] + segments
    if segs2[2] is not None and segs2[2].length_sq > EPS_SQ:
        segments.append(segs2[2])
    return segments


def create_intersecting_segments(pts1, pts2, segs1, segs2, angles, ref):
    """Creates list with the Segments that are visible from ref for the case
    when the two line segments intersect. In counterclockwise order pts1 is
    initially closer to ref, then, after the intersection, pts2 is closer to
    ref.

    Args:
        pts1 (list[list]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            first Segment
        pts2 (list[list]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            second Segment
        segs1 (list[Segment]): list of Segment between the points in pts1
        segs2 (list[Segment]): list of Segment between the points in pts2
        angles (list[float]): list of directions (radians) of the rays from ref
            the angles are obtained from the two extremes of each original
            Segment. The list always has 4 elements (even if repeated)
        ref (list): reference point
    """
    s21 = segs2[1].p1
    s22 = segs2[1].p2
    inters = segs1[1].intersect(s21, [s22[0] - s21[0], s22[1] - s21[1]])[1]
    inters_centered = [inters[0] - ref[0], inters[1] - ref[1]]
    inters_ang = my_atan2(inters_centered[1], inters_centered[0])
    segments = []
    segments.append(Segment(pts1[1], inters, angles[1], inters_ang, ref))
    segments.append(Segment(inters, pts2[2], inters_ang, angles[2], ref))
    if pts1[0] is not None:
        segments[0].merge(Segment(pts1[0], pts1[1], angles[0], angles[1], ref))
    if pts2[3] is not None:
        segments[1].merge(Segment(pts2[2], pts2[3], angles[2], angles[3], ref))
    if segs2[0] is not None and segs2[0].length > EPS:
        segments = [segs2[0]] + segments
    if segs1[2] is not None and segs1[2].length > EPS:
        segments.append(segs1[2])
    return segments


def intersect_segments(n1, n2):
    """Computes list with all Segments that are visible from ref. Assumes
    n1.ref == n2.ref.

    Args:
        n1 (Segment): first Segment
        n2 (Segment): second Segment

    Return:
        visible_segments (list[Segment]): list with all Segments that are
            visible from ref
    """
    if n1.theta2 <= n2.theta1:
        return [n1, n2]
    elif n2.theta2 <= n1.theta1:
        return [n2, n1]

    ref = n1.ref

    d1p1 = dist_sq(n1.p1, ref)
    d1p2 = dist_sq(n1.p2, ref)
    d2p1 = dist_sq(n2.p1, ref)
    d2p2 = dist_sq(n2.p2, ref)
    if max(d1p1, d1p2) < min(d2p1, d2p2) and \
       n1.theta1 <= n2.theta1 and n1.theta2 >= n2.theta2:
            return [n1]
    elif max(d2p1, d2p2) < min(d1p1, d1p2) and \
       n2.theta1 <= n1.theta1 and n2.theta2 >= n1.theta2:
            return [n2]

    angles = sorted([n1.theta1, n1.theta2, n2.theta1, n2.theta2])
    directs = [[math.cos(t), math.sin(t)] for t in angles]
    pts1 = make_pts(n1, directs, angles)
    pts2 = make_pts(n2, directs, angles)
    segs1 = [make_segment(p1, p2, ref) for p1, p2 in zip(pts1[:-1], pts1[1:])]
    segs2 = [make_segment(p1, p2, ref) for p1, p2 in zip(pts2[:-1], pts2[1:])]
    # In theory the intersection (segsX[1]) always exists, but may be too small
    # We don't know about the others (segsX[0] and segsX[2] may be None)
    if segs1[1] is None or segs2[1] is None:
        segs = []
        if segs1[1] is None:
            segs += [seg for seg in segs1 if seg is not None]
        else:
            segs += [n1]
        if segs2[1] is None:
            segs += [seg for seg in segs2 if seg is not None]
        else:
            segs += [n2]
        return segs
    d11 = dist_sq(segs1[1].p1, ref)
    d12 = dist_sq(segs1[1].p2, ref)
    d21 = dist_sq(segs2[1].p1, ref)
    d22 = dist_sq(segs2[1].p2, ref)

    if abs(d11 - d21) < EPS and abs(d12 - d22) < EPS:
        segs = [seg for seg in segs1 + segs2 if seg is not None]
        seg0 = segs[0]
        for seg1 in segs[1:]:
            seg0.merge(seg1)
        return [seg0]
    elif d11 <= d21 and d12 <= d22:
        return create_non_intersecting_segments(pts1, pts2, segs2, angles, ref)
    elif d21 <= d11 and d22 <= d12:
        return create_non_intersecting_segments(pts2, pts1, segs1, angles, ref)
    elif d11 <= d21 and d22 <= d12:
        return create_intersecting_segments(pts1, pts2, segs1, segs2, angles, ref)
    else:
        return create_intersecting_segments(pts2, pts1, segs2, segs1, angles, ref)
