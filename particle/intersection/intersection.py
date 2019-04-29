import math
from collections import namedtuple
import numpy as np

from .visible_segments import create_segments, intersect_segments
from .math_utils import dist


Intersection = namedtuple('Intersection', 'theta, segment, intersection, distance')


def to_key(f):
    return round(f, 5)


class IntersectionFinder:
    def __init__(self, ref=None):
        if ref is None:
            ref = [0, 0]
        self.ref = ref
        # keys are the initial angle
        self._segments = {}
        self._ends = {}

    def add_segments(self, segments):
        for segment in segments:
            self.add_segment(segment)

    def add_segment(self, segment):
        segments = create_segments(segment, self.ref)
        for seg in segments:
            self._add_segment(seg)

    def _add_segment(self, segment):
        ints = self._find_intersections(segment.theta1, segment.theta2)
        if ints:
            for intersection in ints:
                s = to_key(intersection.theta1)
                del self._segments[s]
                del self._ends[s]
                new_segments = intersect_segments(intersection, segment)
                for new_seg in new_segments:
                    self._force_add(new_seg)
                segment = new_segments[-1]
        else:
            self._force_add(segment)

    def _force_add(self, segment):
        s0, s1 = to_key(segment.theta1), to_key(segment.theta2)
        self._segments[s0] = segment
        self._ends[s0] = s1

    def _find_intersections(self, s0, s1):
        segs = []
        for ang in sorted(self._segments.keys()):
            e_ang = self._ends[ang]
            if ang <= s0 <= e_ang or ang <= s1 <= e_ang or \
               s0 <= ang <= s1 or s0 <= e_ang <= s1:
                segs.append(self._segments[ang])
            elif s1 < ang:
                break
        return segs

    def compute_intersections(self, angles=None):
        if angles is None:
            angles = [math.radians(i) for i in range(361)]
        intersections = {}
        segs = self.segments
        seg_iter = iter(segs)
        cur_seg = next(seg_iter, None)
        angles = sorted(angles, key=lambda ang: ang % (2 * math.pi))
        for i in angles:
            angle = i % (2 * math.pi)
            intersections[i] = None
            while cur_seg is not None and cur_seg.theta2 < angle:
                cur_seg = next(seg_iter, None)
            if cur_seg is None:
                break

            if cur_seg.theta1 <= angle <= cur_seg.theta2:
                direction = [math.cos(angle), math.sin(angle)]
                exists, int_pt = cur_seg.intersect(self.ref, direction)
                if exists:
                    new_int = Intersection(angle, cur_seg, int_pt, dist(self.ref, int_pt))
                    intersections[i] = new_int

        return intersections

    @property
    def segments(self):
        return list(sorted(self._segments.values(), key=lambda s: s.theta1))


def find_intersections(p, segments, angles=None):
    """
    Finds the closest intersection between rays starting at p and the line segments.

    Args:
        p (list): rays' starting point (x, y)
        segments (list[Segment]): list of line segments (Segment)
        step (float): step between rays (degrees). Default: 1

    Return:
       list[Intersection], list[Segment]: intersections and visible segments
    """

    if not segments:
        return [], []
    if angles is None:
        angles = [math.radians(i) for i in range(361)]
    finder = IntersectionFinder(segments[0].ref)
    for seg in segments:
        finder.add_segment(seg)
    return finder.compute_intersections(angles), finder.segments
