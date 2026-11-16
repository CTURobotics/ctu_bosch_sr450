#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from unittest import TestCase
import numpy as np

from ctu_bosch_sr450.circle_circle_intersection import circle_circle_intersection


class TestCircleCircleIntersection(TestCase):
    def test_circle_circle_intersection(self):
        """Test circle_circle_intersection function."""

        ints = circle_circle_intersection([0, 0], 1, [1, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        if by > ay:
            ay, by = by, ay
        self.assertAlmostEqual(ax, 0.5)
        self.assertAlmostEqual(ay, np.sqrt(1 - 0.5**2))
        self.assertAlmostEqual(bx, 0.5)
        self.assertAlmostEqual(by, -np.sqrt(1 - 0.5**2))

        ints = circle_circle_intersection([0, 0], 1, [2, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        self.assertAlmostEqual(ax, 1.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 0.0)

        ints = circle_circle_intersection([0, 0], 1, [0, 0], 1)
        self.assertEqual(len(ints), 2)
        a, b = ints
        self.assertAlmostEqual(np.linalg.norm(a), 1.0)
        self.assertAlmostEqual(np.linalg.norm(b), 1.0)

    def _assert_on_both_circles(self, ints, c0, r0, c1, r1, atol=1e-9):
        for p in ints:
            self.assertAlmostEqual(np.linalg.norm(p - c0), r0, delta=atol)
            self.assertAlmostEqual(np.linalg.norm(p - c1), r1, delta=atol)

    def test_no_intersection(self):
        """Circles apart or one strictly inside the other: empty list."""
        self.assertEqual(len(circle_circle_intersection([0, 0], 1, [5, 0], 1)), 0)
        self.assertEqual(len(circle_circle_intersection([0, 0], 3, [0.5, 0], 1)), 0)
        self.assertEqual(len(circle_circle_intersection([0.5, 0], 1, [0, 0], 3)), 0)
        # concentric with different radii
        self.assertEqual(len(circle_circle_intersection([0, 0], 1, [0, 0], 2)), 0)

    def test_tangent_rounding(self):
        """Exactly tangent circles whose rounding makes r0**2 - a**2 negative."""
        c0, r0 = np.array([-0.766735510274243, 3.2770259382044173]), 2.514274904578052
        c1, r1 = np.array([5.002554891197656, 4.286136541335328]), 3.3426027572710515
        self.assertAlmostEqual(np.linalg.norm(c1 - c0), r0 + r1)
        ints = circle_circle_intersection(c0, r0, c1, r1)
        self.assertEqual(len(ints), 2)
        self._assert_on_both_circles(ints, c0, r0, c1, r1)
        np.testing.assert_allclose(ints[0], ints[1], atol=1e-6)

    def test_internal_tangent(self):
        """Smaller circle touching the bigger one from inside."""
        ints = circle_circle_intersection([0, 0], 3, [1, 0], 2)
        self.assertEqual(len(ints), 2)
        for p in ints:
            np.testing.assert_allclose(p, [3, 0], atol=1e-9)

    def test_tangent_random(self):
        """Random external and internal tangency: a single (duplicated) point."""
        rng = np.random.default_rng(0)
        for _ in range(2000):
            c0 = rng.uniform(-5, 5, 2)
            r0, r1 = rng.uniform(0.1, 6, 2)
            ang = rng.uniform(0, 2 * np.pi)
            for dist in (r0 + r1, abs(r0 - r1)):
                c1 = c0 + dist * np.array([np.cos(ang), np.sin(ang)])
                ints = circle_circle_intersection(c0, r0, c1, r1)
                self.assertEqual(len(ints), 2)
                self._assert_on_both_circles(ints, c0, r0, c1, r1, atol=1e-6)
                np.testing.assert_allclose(ints[0], ints[1], atol=1e-3)

    def test_random(self):
        """Random pairs: solution count matches geometry, points lie on both."""
        rng = np.random.default_rng(1)
        for _ in range(2000):
            c0, c1 = rng.uniform(-5, 5, 2), rng.uniform(-5, 5, 2)
            r0, r1 = rng.uniform(0.1, 6, 2)
            d = np.linalg.norm(c1 - c0)
            if abs(d - (r0 + r1)) < 1e-6 or abs(d - abs(r0 - r1)) < 1e-6:
                continue  # tangency handled by the dedicated test
            ints = circle_circle_intersection(c0, r0, c1, r1)
            expected = 0 if (d > r0 + r1 or d < abs(r0 - r1)) else 2
            self.assertEqual(len(ints), expected)
            self._assert_on_both_circles(ints, c0, r0, c1, r1)
            if ints:
                # the two points are mirror images across the line of centers
                u = (c1 - c0) / d
                m = (ints[0] + ints[1]) / 2 - c0
                self.assertAlmostEqual(u[0] * m[1] - u[1] * m[0], 0.0, delta=1e-9)
