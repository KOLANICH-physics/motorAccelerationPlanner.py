#!/usr/bin/env python3
import itertools
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import motorAccelerationPlanner
from motorAccelerationPlanner import ArbitraryPositionChangePlan, Limits, PositiveSpeedPositionChangePlan, State


class TestsPositive(unittest.TestCase):
	def _testPositivePathUsingMatrix(self, matrix):
		for speedLimit, path, initialSpeed, finalSpeed in matrix:
			with self.subTest(speedLimit=speedLimit, path=path, initialSpeed=initialSpeed, finalSpeed=finalSpeed):
				limits = Limits(speed=speedLimit, accel=3, deccel=5)
				sched = PositiveSpeedPositionChangePlan.compute(path, limits, initialSpeed, finalSpeed)
				resPath = sched.position(limits, initialSpeed)
				self.assertAlmostEqual(path, resPath)

	def testPathPositiveNonTruncated(self):
		self._testPositivePathUsingMatrix(
			(
				(25, 90, 0, 0),
				(25, 90, 10, 0),
				(25, 90, 0, 10),
				(25, 90, 15, 10),
			)
		)

	def testPathPositiveTruncated(self):
		self._testPositivePathUsingMatrix(
			(
				(15, 90, 0, 0),
				(15, 90, 10, 0),
				(15, 90, 0, 10),
				(15, 90, 15, 10),
			)
		)


class TestsArbitrary(TestsPositive):
	def _testPositivePathUsingMatrix(self, matrix):
		for speedLimit, path, initialSpeed, finalSpeed in matrix:
			initial = State(position=0, speed=initialSpeed)
			final = State(position=path, speed=finalSpeed)
			with self.subTest(speedLimit=speedLimit, path=path, initialSpeed=initialSpeed, finalSpeed=finalSpeed, initialPosition=initial.position, finalPosition=final.position):
				limits = Limits(speed=speedLimit, accel=20, deccel=200)
				sched = ArbitraryPositionChangePlan.compute(initial, final, limits)
				resPath = sched.position(limits, initial)
				self.assertEqual(sched.tPreDeccel, 0)
				self.assertEqual(sched.tPostAccel, 0)
				self.assertAlmostEqual(final.position, resPath)


if __name__ == "__main__":
	unittest.main()
