import math
from pytest import approx
import numpy as np

from visible_segments import VisibleSegments, Segment, create_segments, intersect_segments


def test_merge_before():
    pts = [
        np.array([1, 1]),
        np.array([0, 1]),
        np.array([-1, 1]),
    ]
    angles = [
        math.pi/4,
        math.pi/2,
        3*math.pi/4,
    ]
    seg1 = Segment(pts[0], pts[1], angles[0], angles[1])
    seg2 = Segment(pts[1], pts[2], angles[1], angles[2])
    seg2.merge(seg1)
    np.testing.assert_allclose(seg2.p1, pts[0])
    np.testing.assert_allclose(seg2.p2, pts[2])
    assert seg2.theta1 == approx(angles[0])
    assert seg2.theta2 == approx(angles[2])


def test_merge_after():
    pts = [
        np.array([1, 1]),
        np.array([0, 1]),
        np.array([-1, 1]),
    ]
    angles = [
        math.pi/4,
        math.pi/2,
        3*math.pi/4,
    ]
    seg1 = Segment(pts[0], pts[1], angles[0], angles[1])
    seg2 = Segment(pts[1], pts[2], angles[1], angles[2])
    seg1.merge(seg2)
    np.testing.assert_allclose(seg1.p1, pts[0])
    np.testing.assert_allclose(seg1.p2, pts[2])
    assert seg1.theta1 == approx(angles[0])
    assert seg1.theta2 == approx(angles[2])


def test_merge_none():
    pts = [
        np.array([1, 1]),
        np.array([0, 1]),
    ]
    angles = [
        math.pi/4,
        math.pi/2,
    ]
    seg1 = Segment(pts[0], pts[1], angles[0], angles[1])
    seg1.merge(None)
    np.testing.assert_allclose(seg1.p1, pts[0])
    np.testing.assert_allclose(seg1.p2, pts[1])
    assert seg1.theta1 == approx(angles[0])
    assert seg1.theta2 == approx(angles[1])


def test_intersection():
    orig = np.array([1, 1])
    p1 = np.array([1, 0])
    p2 = np.array([0, 1])
    seg = Segment(p1, p2)
    tests = [
        (np.array([-1, -1]), True, np.array([0.5, 0.5])),
        (np.array([-1, 0]), True, np.array([0, 1])),
        (np.array([-1, 1]), False, None),
        (np.array([0, 1]), False, None),
        (np.array([1, 1]), False, None),
        (np.array([1, 0]), False, None),
        (np.array([1, -1]), False, None),
        (np.array([0, -1]), True, np.array([1, 0])),
    ]
    for direct, intersects, intersection in tests:
        intersected, detected_intersection = seg.intersect(orig, direct)
        assert intersects == intersected
        if intersection is None:
            assert detected_intersection is None
        else:
            np.testing.assert_almost_equal(detected_intersection, intersection)


def test_create_nodes():
    p1 = np.array([1, 0])
    p2 = np.array([1, 1])
    nodes = create_segments(Segment(p1, p2))
    assert len(nodes) == 1
    np.testing.assert_almost_equal(nodes[0].p1, p1)
    np.testing.assert_almost_equal(nodes[0].p2, p2)
    assert nodes[0].theta1 == approx(0)
    assert nodes[0].theta2 == approx(math.pi / 4)


def test_create_nodes_always_counterclockwise():
    p1 = np.array([1, 0])
    p2 = np.array([1, -1])
    nodes = create_segments(Segment(p1, p2))
    assert len(nodes) == 1
    np.testing.assert_almost_equal(nodes[0].p1, p2)
    np.testing.assert_almost_equal(nodes[0].p2, p1)
    assert nodes[0].theta1 == approx(7 * math.pi / 4)
    assert nodes[0].theta2 == approx(2 * math.pi)


def test_create_nodes_with_origin():
    p1 = np.array([1, 0])
    p2 = np.array([1, 1])
    ref = np.array([2, 1])
    nodes = create_segments(Segment(p1, p2), ref)
    assert len(nodes) == 1
    np.testing.assert_almost_equal(nodes[0].p1, p2)
    np.testing.assert_almost_equal(nodes[0].p2, p1)
    assert nodes[0].theta1 == approx(math.pi)
    assert nodes[0].theta2 == approx(5 * math.pi / 4)
    np.testing.assert_almost_equal(nodes[0].ref, ref)


def test_split_segment_when_crossing_origin():
    p1 = np.array([1, -1])
    p2 = np.array([1, 1])
    p3 = np.array([1, 0])
    nodes = create_segments(Segment(p1, p2))
    assert len(nodes) == 2
    np.testing.assert_almost_equal(nodes[0].p1, p3)
    np.testing.assert_almost_equal(nodes[0].p2, p2)
    np.testing.assert_almost_equal(nodes[1].p1, p1)
    np.testing.assert_almost_equal(nodes[1].p2, p3)


def test_dont_split_segment_on_negative_x():
    p1 = np.array([-1, 1])
    p2 = np.array([-1, -1])
    nodes = create_segments(Segment(p1, p2))
    assert len(nodes) == 1
    np.testing.assert_almost_equal(nodes[0].p1, p1)
    np.testing.assert_almost_equal(nodes[0].p2, p2)
    assert nodes[0].theta1 == approx(3 * math.pi / 4)
    assert nodes[0].theta2 == approx(5 * math.pi / 4)


def test_non_angle_intersecting_nodes():
    n1 = Segment(np.array([1, 1]), np.array([-1, 1]), math.pi/4, 3*math.pi/4)
    n2 = Segment(np.array([-1, -1]), np.array([1, -1]), 5*math.pi/4, 7*math.pi/4)
    nodes = intersect_segments(n1, n2)
    assert len(nodes) == 2
    assert nodes[0] == n1
    assert nodes[1] == n2
    nodes = intersect_segments(n2, n1)
    assert len(nodes) == 2
    assert nodes[0] == n1
    assert nodes[1] == n2


def test_non_angle_intersecting_nodes_other_origin():
    ref = np.array([10, 10])
    n1 = Segment(np.array([1, 1]) + ref,
              np.array([-1, 1]) + ref,
              math.pi/4, 3*math.pi/4, ref)
    n2 = Segment(np.array([-1, -1]) + ref,
              np.array([1, -1]) + ref,
              5*math.pi/4, 7*math.pi/4, ref)
    nodes = intersect_segments(n1, n2)
    assert len(nodes) == 2
    assert nodes[0] == n1
    assert nodes[1] == n2
    nodes = intersect_segments(n2, n1)
    assert len(nodes) == 2
    assert nodes[0] == n1
    assert nodes[1] == n2


def test_segment_hidden():
    n1 = Segment(np.array([1, 1]), np.array([-1, 1]), math.pi/4, 3*math.pi/4)
    n2 = Segment(np.array([2, 2]), np.array([-2, 2]), math.pi/4, 3*math.pi/4)
    nodes = intersect_segments(n1, n2)
    assert len(nodes) == 1
    assert nodes[0] == n1

    nodes = intersect_segments(n2, n1)
    assert len(nodes) == 1
    assert nodes[0] == n1


def test_intersecting_segments():
    n1 = Segment(np.array([1, 1]), np.array([-2, 2]), math.pi/4, 3*math.pi/4)
    n2 = Segment(np.array([2, 2]), np.array([-1, 1]), math.pi/4, 3*math.pi/4)
    retn1 = Segment(np.array([1, 1]), np.array([0, 4/3]), math.pi/4, math.pi/2)
    retn2 = Segment(np.array([0, 4/3]), np.array([-1, 1]), math.pi/2, 3*math.pi/4)
    nodes = intersect_segments(n1, n2)
    assert len(nodes) == 2
    assert nodes[0] == retn1
    assert nodes[1] == retn2

    nodes = intersect_segments(n2, n1)
    assert len(nodes) == 2
    assert nodes[0] == retn1
    assert nodes[1] == retn2


def test_add_segments():
    visible = VisibleSegments()
    segment = Segment(np.array([1, 0]), np.array([1, 1]))
    visible.add_segment(segment)
    segments = visible.segments
    assert len(segments) == 1
    assert segments[0] == segment


def test_add_two_segments():
    visible = VisibleSegments()
    segment1 = Segment(np.array([1, 0]), np.array([1, 1]))
    segment2 = Segment(np.array([-1, 0]), np.array([-1, -1]))
    visible.add_segment(segment2)
    visible.add_segment(segment1)
    segments = visible.segments
    assert len(segments) == 2
    assert segments[0] == segment1
    assert segments[1] == segment2

def test_add_split():
    visible = VisibleSegments()
    p1 = np.array([1, -1])
    p2 = np.array([1, 0])
    p3 = np.array([1, 1])
    segment = Segment(p3, p1)
    visible.add_segment(segment)
    segments = visible.segments
    assert len(segments) == 2
    assert segments[0] == Segment(p2, p3)
    assert segments[1] == Segment(p1, p2)

def test_add_square():
    visible = VisibleSegments()
    angles = [
        0,
        math.pi/4,
        3*math.pi/4,
        5*math.pi/4,
        7*math.pi/4,
        2*math.pi,
    ]
    pts = [
        np.array([1, 0]),
        np.array([1, 1]),
        np.array([-1, 1]),
        np.array([-1, -1]),
        np.array([1, -1]),
    ]
    segs = [Segment(p1, p2) for p1, p2 in zip(pts, pts[1:] + pts[0:1])]
    visible.add_segments(segs)
    segments = visible.segments
    assert len(segments) == 5
    for s1, s2 in zip(segments, segs):
        assert s1 == s2

def test_add_intersecting_segments():
    visible = VisibleSegments()
    p1 = np.array([1, 1])
    p2 = np.array([-2, 1])
    p3 = np.array([-1, 2])
    p4 = np.array([-1, -1])
    p5 = np.array([-1, 1])
    visible.add_segment(Segment(p1, p2))
    segments = visible.segments
    assert len(segments) == 1
    assert segments[0] == Segment(p1, p2)
    visible.add_segment(Segment(p3, p4))
    segments = visible.segments
    assert len(segments) == 2
    assert segments[0] == Segment(p1, p5)
    assert segments[1] == Segment(p5, p4)


def test_add_intersecting_segments_inverse():
    visible = VisibleSegments()
    p1 = np.array([1, 1])
    p2 = np.array([-2, 1])
    p3 = np.array([-1, 2])
    p4 = np.array([-1, -1])
    p5 = np.array([-1, 1])
    visible.add_segment(Segment(p3, p4))
    segments = visible.segments
    assert len(segments) == 1
    assert segments[0] == Segment(p3, p4)
    visible.add_segment(Segment(p1, p2))
    segments = visible.segments
    assert len(segments) == 2
    assert segments[0] == Segment(p1, p5)
    assert segments[1] == Segment(p5, p4)


def test_add_intersecting_segments_first_farther():
    ref = np.array([10, -100])
    visible = VisibleSegments(ref)
    segs = [
        Segment(np.array([-3, 3]) + ref, np.array([1, 0]) + ref, ref=ref),
        Segment(np.array([3, 3]) + ref, np.array([-1, 0]) + ref, ref=ref),
    ]
    exp = [
        Segment(np.array([1, 0]) + ref, np.array([0, 0.75]) + ref, ref=ref),
        Segment(np.array([0, 0.75]) + ref, np.array([-1, 0]) + ref, ref=ref),
    ]
    visible.add_segments(segs)
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected

    # Inverse
    visible = VisibleSegments(ref)
    visible.add_segments(segs[::-1])
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected


def test_intersecting_square():
    visible = VisibleSegments()
    visible.add_segments([
        Segment(np.array([-2, 1]), np.array([2, 1])),
        Segment(np.array([-2, -1]), np.array([2, -1])),
        Segment(np.array([1, 2]), np.array([1, -2])),
        Segment(np.array([-1, 2]), np.array([-1, -2])),
    ])
    segs = [
        Segment(np.array([1, 0]), np.array([1, 1])),
        Segment(np.array([1, 1]), np.array([-1, 1])),
        Segment(np.array([-1, 1]), np.array([-1, -1])),
        Segment(np.array([-1, -1]), np.array([1, -1])),
        Segment(np.array([1, -1]), np.array([1, 0])),
    ]
    for segment, expected in zip(visible.segments, segs):
        assert segment == expected


def test_intersecting_limit():
    ref = np.array([10, -100])
    visible = VisibleSegments(ref)
    segs = [
        Segment(np.array([1, 1]) + ref, np.array([-1, 1]) + ref, ref=ref),
        Segment(np.array([-1, 1]) + ref, np.array([-1, -1]) + ref, ref=ref),
    ]
    visible.add_segments(segs)
    for segment, expected in zip(visible.segments, segs):
        assert segment == expected

    # Inverse
    visible = VisibleSegments(ref)
    visible.add_segments(segs[::-1])
    for segment, expected in zip(visible.segments, segs):
        assert segment == expected


def test_intersecting_first():
    ref = np.array([10, -100])
    visible = VisibleSegments(ref)
    segs = [
        Segment(np.array([1, 1]) + ref, np.array([-2, 1]) + ref, ref=ref),
        Segment(np.array([-1, 1]) + ref, np.array([-1, -1]) + ref, ref=ref),
    ]
    exp = [
        Segment(np.array([1, 1]) + ref, np.array([-1, 1]) + ref, ref=ref),
        segs[1],
    ]
    visible.add_segments(segs)
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected

    # Inverse
    visible = VisibleSegments(ref)
    visible.add_segments(segs[::-1])
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected


def test_intersecting_second():
    ref = np.array([10, -100])
    visible = VisibleSegments(ref)
    segs = [
        Segment(np.array([1, 1]) + ref, np.array([-1, 1]) + ref, ref=ref),
        Segment(np.array([-1, 2]) + ref, np.array([-1, -1]) + ref, ref=ref),
    ]
    exp = [
        segs[0],
        Segment(np.array([-1, 1]) + ref, np.array([-1, -1]) + ref, ref=ref),
    ]
    visible.add_segments(segs)
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected

    # Inverse
    visible = VisibleSegments(ref)
    visible.add_segments(segs[::-1])
    for segment, expected in zip(visible.segments, exp):
        assert segment == expected

