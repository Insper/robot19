import math
import bisect
import attr
import numpy as np

from math_utils import EPS, dist_sq, my_atan2
from segment import Segment


def create_segments(segment, ref=None):
    """Creates Segment list breaking the segment if direction [1, 0] intersects
    it. This function is used to avoid the case in which the points are sorted
    counterclockwise with reference to ref and theta1 > theta2.
    """
    if ref is None:
        ref = np.array([0, 0])

    p1 = segment.p1
    p2 = segment.p2
    x1 = p1[0] - ref[0]
    y1 = p1[1] - ref[1]
    x2 = p2[0] - ref[0]
    y2 = p2[1] - ref[1]

    if x1 * y2 - x2 * y1 < 0:
        p1, p2 = p2, p1
        x1, y1, x2, y2 = x2, y2, x1, y1

    theta1 = my_atan2(y1, x1)
    theta2 = my_atan2(y2, x2)

    hor = np.array([1, 0])
    intersects, intersection = segment.intersect(ref, hor)
    if intersects:
        if abs(theta2) < EPS and abs(theta1) > EPS:
            return [Segment(p1, p2, theta1, 2 * math.pi, ref)]
        elif abs(theta1) < EPS and abs(theta2) > EPS:
            return [Segment(p1, p2, 0, theta2, ref)]
        return [
            Segment(intersection, p2, 0, theta2, ref),
            Segment(p1, intersection, theta1, 2 * math.pi, ref),
        ]
    return [Segment(p1, p2, theta1, theta2, ref)]


def make_segment(p1, p2, ref=None):
    """Auxiliary function that either creates a Segment or returns None if
    arguments are not valid.
    """
    if p1 is not None and p2 is not None and not np.allclose(p1, p2):
        return Segment(p1, p2, ref=ref)
    return None


def create_non_intersecting_segments(pts1, pts2, segs2, angles, ref):
    """Creates list with the Segments that are visible from ref for the case
    when the line segment in pts1 is in front of the line segment in pts2.

    Args:
        pts1 (list[np.array]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            Segment closest to ref
        pts2 (list[np.array]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            Segment farthest to ref
        segs2 (list[Segment]): list of Segment between the points in pts2
        angles (list[float]): list of directions (radians) of the rays from ref
            the angles are obtained from the two extremes of each original
            Segment. The list always has 4 elements (even if repeated)
        ref (np.array): reference point
    """
    segments = []
    segment = Segment(pts1[1], pts1[2], angles[1], angles[2], ref)
    if pts1[0] is not None:
        segment.merge(Segment(pts1[0], pts1[1], angles[0], angles[1], ref))
    if pts1[3] is not None:
        segment.merge(Segment(pts1[2], pts1[3], angles[2], angles[3], ref))
    segments.append(segment)
    if segs2[0] is not None:
        segments = [Segment(pts2[0], pts2[1], angles[0], angles[1], ref)] + segments
    if segs2[2] is not None:
        segments.append(Segment(pts2[2], pts2[3], angles[2], angles[3], ref))
    return segments


def create_intersecting_segments(pts1, pts2, segs1, segs2, angles, ref):
    """Creates list with the Segments that are visible from ref for the case
    when the two line segments intersect. In counterclockwise order pts1 is
    initially closer to ref, then, after the intersection, pts2 is closer to
    ref.

    Args:
        pts1 (list[np.array]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            first Segment
        pts2 (list[np.array]): list of 2D points that are the intersection of
            the rays from ref in the directions indicated by angles with the
            second Segment
        segs1 (list[Segment]): list of Segment between the points in pts1
        segs2 (list[Segment]): list of Segment between the points in pts2
        angles (list[float]): list of directions (radians) of the rays from ref
            the angles are obtained from the two extremes of each original
            Segment. The list always has 4 elements (even if repeated)
        ref (np.array): reference point
    """
    inters = segs1[1].intersect(segs2[1].p1, segs2[1].p2-segs2[1].p1)[1]
    inters_centered = inters - ref
    inters_ang = my_atan2(inters_centered[1], inters_centered[0])
    segments = []
    segments.append(Segment(pts1[1], inters, angles[1], inters_ang, ref))
    segments.append(Segment(inters, pts2[2], inters_ang, angles[2], ref))
    if pts1[0] is not None:
        segments[0].merge(Segment(pts1[0], pts1[1], angles[0], angles[1], ref))
    if pts2[3] is not None:
        segments[1].merge(Segment(pts2[2], pts2[3], angles[2], angles[3], ref))
    if segs2[0] is not None:
        segments = [Segment(pts2[0], pts2[1], angles[0], angles[1], ref)] + segments
    if segs1[2] is not None:
        segments.append(Segment(pts1[2], pts1[3], angles[2], angles[3], ref))
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
    # Just to make sure things are ok
    np.testing.assert_allclose(n1.ref, n2.ref)
    ref = n1.ref
    angles = sorted([n1.theta1, n1.theta2, n2.theta1, n2.theta2])
    directs = [np.array([math.cos(t), math.sin(t)]) for t in angles]
    pts1 = [n1.intersect(ref, d)[1] for d in directs]
    pts2 = [n2.intersect(ref, d)[1] for d in directs]
    segs1 = [make_segment(p1, p2, ref) for p1, p2 in zip(pts1[:-1], pts1[1:])]
    segs2 = [make_segment(p1, p2, ref) for p1, p2 in zip(pts2[:-1], pts2[1:])]
    # The intersection (segsX[1]) always exists
    # But we don't know about the others (segsX[0] and segsX[2] may be None)
    d11 = dist_sq(segs1[1].p1, ref)
    d12 = dist_sq(segs1[1].p2, ref)
    d21 = dist_sq(segs2[1].p1, ref)
    d22 = dist_sq(segs2[1].p2, ref)

    if d11 <= d21 and d12 <= d22:
        return create_non_intersecting_segments(pts1, pts2, segs2, angles, ref)
    elif d21 <= d11 and d22 <= d12:
        return create_non_intersecting_segments(pts2, pts1, segs1, angles, ref)
    elif d11 <= d21 and d22 <= d12:
        return create_intersecting_segments(pts1, pts2, segs1, segs2, angles, ref)
    else:
        return create_intersecting_segments(pts2, pts1, segs2, segs1, angles, ref)


@attr.s
class VisibleSegments:
    """Data structure that keeps an updated list that contains only the
    Segments that are visible from ref.

    Attributes:
        ref (np.array): 2D reference point
        segments (list[Segment]): list with all Segments that are visible from
            ref
    """
    _ref = attr.ib(default=attr.Factory(lambda: np.array([0, 0])))
    segments = attr.ib(default=attr.Factory(list))

    def add_segments(self, segments):
        """Add list of segments at once.

        Args:
            segments (list[Segment]): list of Segments to be added
        """
        for segment in segments:
            self.add_segment(segment)

    def add_segment(self, segment):
        """Add single Segment.

        Args:
            segment (Segment): Segment to be added
        """
        new_segments = create_segments(segment, self._ref)
        for seg in new_segments:
            self._add_segment_r(seg)

    def _readd_intersecting(self, idx, segment):
        previous = self.segments[idx]
        del self.segments[idx]
        new_segs = intersect_segments(segment, previous)
        for seg in new_segs:
            self._add_segment_r(seg)

    def _add_segment_r(self, segment):
        i = bisect.bisect(self.segments, segment)
        if i > 0 and self.segments[i-1].theta2 > segment.theta1:
            self._readd_intersecting(i-1, segment)
        elif i < len(self.segments) - 1 and \
            self.segments[i+1].theta1 < segment.theta2:
            self._readd_intersecting(i+1, segment)
        else:
            self.segments.insert(i, segment)

