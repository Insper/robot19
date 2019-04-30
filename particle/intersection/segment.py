import math
import random
import attr
import numpy as np

from .math_utils import EPS, THETA_EPS, my_atan2, dist, dist_sq


@attr.s(cmp=False)
class Segment(object):
    """Line segment.

    Attributes:
        p1 (list): starting point
        p2 (list): ending point
        theta1 (float): counterclockwise rotation from vector [1, 0] to p1
            using ref as the origin
        theta2 (float): counterclockwise rotation from vector [1, 0] to p2
            using ref as the origin
        ref (list): reference point to be used by the algorithm
    """

    p1 = attr.ib()
    p2 = attr.ib()
    _theta1 = attr.ib(default=None)
    _theta2 = attr.ib(default=None)
    _ref = attr.ib(default=attr.Factory(lambda: [0, 0]))

    @property
    def theta1(self):
        if self._theta1 is None:
            self._theta1 = my_atan2(self.p1[1] - self.ref[1],
                                    self.p1[0] - self.ref[0])
        return self._theta1

    @theta1.setter
    def theta1(self, t1):
        self._theta1 = t1

    @property
    def theta2(self):
        if self._theta2 is None:
            self._theta2 = my_atan2(self.p2[1] - self.ref[1],
                                    self.p2[0] - self.ref[0])
            if self._theta2 < EPS and self._theta2 < self.theta1:
                self._theta2 = 2 * math.pi
        return self._theta2

    @theta2.setter
    def theta2(self, t2):
        self._theta2 = t2

    @property
    def ref(self):
        return self._ref

    @ref.setter
    def ref(self, ref):
        self.ref = ref
        self._theta1 = None
        self._theta2 = None

    @property
    def length(self):
        return dist(self.p1, self.p2)

    @property
    def length_sq(self):
        return dist_sq(self.p1, self.p2)

    def __eq__(self, other):
        if other.__class__ is not self.__class__:
            return NotImplemented
        return np.allclose(self.p1, other.p1) and \
               np.allclose(self.p2, other.p2) and \
               abs(self.theta1 - other.theta1) < THETA_EPS and \
               abs(self.theta2 - other.theta2) < THETA_EPS and \
               np.allclose(self.ref, other.ref)

    def __hash__(self):
        return hash((tuple(self.p1), tuple(self.p2)))

    def __lt__(self, other):
        return self.theta2 < other.theta1 + THETA_EPS

    def __le__(self, other):
        return self.theta2 <= other.theta1 + THETA_EPS

    def __gt__(self, other):
        return self.theta1 + THETA_EPS > other.theta2

    def __ge__(self, other):
        return self.theta1 + THETA_EPS >= other.theta2

    def merge(self, other):
        """Incorporates other Segment into self.
        Assumes the two segments are collinear.

        Args:
            other (Segment): segment to be merged
        """
        if other is None:
            return
        if self.theta1 > other.theta1:
            self.theta1 = other.theta1
            self.p1 = other.p1
        if self.theta2 < other.theta2:
            self.theta2 = other.theta2
            self.p2 = other.p2

    def intersect(self, orig, direct, angle=None):
        """Finds intersection between ray and self.

        Args:
            orig (list): ray's starting point
            direct (list): ray's direction

        Return:
            exists, intersection: exists is a boolean indicating if the
            intersection point exists.
        """
        # Shortcut
        if angle is not None and (angle < self.theta1 or angle > self.theta2):
            return False, None

        # Init vars
        nsq = direct[0]**2 + direct[1]**2
        if abs(nsq - 1) > EPS:
            norm = math.sqrt(nsq)
            direct = [direct[0] / norm, direct[1] / norm]
        ctheta, stheta = direct
        px, py = orig
        x1, y1 = self.p1
        x2, y2 = self.p2

        # Compute s and r
        denom = ((x2 - x1) * stheta + (y1 - y2) * ctheta)
        if abs(denom) < EPS:
            return False, None
        s = ((px - x1) * stheta + (y1 - py) * ctheta) / denom
        if abs(ctheta) > abs(stheta):
            r = (x1 + s * (x2 - x1) - px) / ctheta
        else:
            r = (y1 + s * (y2 - y1) - py) / stheta

        if r < -EPS or s < -EPS or s > 1+EPS:
            return False, None
        return True, [px + r * ctheta, py + r * stheta]


def random_segment(origin, min_value=0, max_value=10, min_len=1, max_len=10):
    p1 = [random.uniform(min_value, max_value), random.uniform(min_value, max_value)]
    theta = random.uniform(0, 2*math.pi)
    length = random.uniform(min_len, max_len)
    p2 = [p1[0] + length*math.cos(theta), p1[1] + length*math.sin(theta)]
    return Segment(p1, p2, ref=origin)
