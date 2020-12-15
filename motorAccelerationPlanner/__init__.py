from dataclasses import dataclass
from math import sqrt


@dataclass
class Limits:
	speed: float
	accel: float
	deccel: float


@dataclass
class State:
	position: float = 0
	speed: float = 0


def signBit(a):
	return a >= 0


def sign(a):
	return 1 if signBit(a) else -1


def areTheSameSign(a: int, b: int) -> bool:
	hasZero = a == 0 or b == 0
	return hasZero or signBit(a) == signBit(b)


def computeAccel(sp, deltaLimit):
	spSign = sign(sp)
	spAbs = sp * spSign
	spAbs = min(spAbs, deltaLimit)
	return spAbs * spSign


def pathAccel(initialSpeed: float, acceleration: float, t: float) -> float:
	return initialSpeed * t + acceleration * t**2 / 2


def pathAccelDeccel(initialSpeed: float, acceleration: float, tAccel: float, decceleration: float, tDeccel: float) -> float:
	"""Computes position given acceleration and decelleration. C++ compiler should be able to guess how to optimize this func, so we keep it as in formula."""

	return pathAccel(initialSpeed, acceleration, tAccel) + pathAccel(initialSpeed + acceleration * tAccel, -decceleration, tDeccel)


@dataclass
class SpeedChangePlan:
	accelerationEpochs: int = 0
	acceleration: int = 0
	residualEpoch: int = 0

	@classmethod
	def compute(cls, currentSpeed: int, setSpeed: int, limits: Limits):
		assert areTheSameSign(currentSpeed, setSpeed)
		speedDelta = setSpeed - currentSpeed
		if abs(currentSpeed) > abs(setSpeed):
			deltaLimit = limits.deccel
		else:
			deltaLimit = limits.accel

		return cls(
			acceleration=computeAccel(speedDelta, deltaLimit),
			accelerationEpochs=speedDelta // accel,
			residualEpoch=speedDelta % accel,
		)


@dataclass
class PositiveSpeedPositionChangePlan:
	"""Position change in picewise fashion. NOT QUANTIZED TO STEPS!
	ASSUMMES THAT ALL THE SPEEDS AND ACCELERATIONS AND DECCELERATIONS ARE POSITIVE.
	0. Execute microops from preamble. They will set the speed to 0 if needed.
	1. accelerating with `limits.accel` acceleration for `tAccel` units of time
	2. steady speed for `tSteady` units of time
	3. decellerating with `limits.deccel` acceleration for `tDeccel` units of time"""

	tAccel: float
	tSteady: float
	tDeccel: float

	@property
	def t1(self):
		return self.tAccel

	@property
	def t2(self):
		return self.t1 + self.tSteady

	@property
	def tTotal(self):
		return self.t2 + self.tDeccel

	def accel(self, t: float, limits: Limits) -> float:
		"""A piecewise acceleration function"""
		if t <= 0:
			return 0

		if t <= self.tAccel:
			return limits.accel

		if t <= self.t2:
			return 0

		if t > self.tTotal:
			return 0

		return -limits.deccel

	def position(self, limits: Limits, initialSpeed: float) -> float:
		return pathAccelDeccel(initialSpeed, limits.accel, self.tAccel, limits.deccel, self.tDeccel) + self.tSteady * self.maxSpeed(limits, initialSpeed)

	def maxSpeed(self, limits: Limits, initialSpeed: float) -> float:
		return initialSpeed + limits.accel * self.tAccel

	def finalSpeed(self, limits: Limits, initialSpeed: float):
		return self.maxSpeed(limits, initialSpeed) - self.tDeccel * limits.deccel

	@classmethod
	def compute(cls, S, limits: Limits, initialSpeed: float = 0, finalSpeed: float = 0) -> "PositiveSpeedPositionChangePlan":
		"""Computes the timings for the movement with the maximal allowed accelerations and speeds (and so, for minimal time)"""

		tRollBackAccelPhase = 0
		if initialSpeed:
			# HACK: reducing the case to the case of zero speed.
			# rolling back the position in time
			# accelerating (reverse in time) with the maximum acceleration allowed,
			# since we always accelerate with the maximum acceleration
			tRollBackAccelPhase = abs(initialSpeed) / limits.accel
			rolledBackPath = limits.accel * tRollBackAccelPhase**2 / 2
			S += rolledBackPath

		tEarlyStopDeccelPhase = 0
		if finalSpeed:
			# the same HACK
			tEarlyStopDeccelPhase = abs(finalSpeed) / limits.deccel
			overshootedPath = limits.deccel * tEarlyStopDeccelPhase**2 / 2
			S += overshootedPath

		res = cls.computeFromRest(S, limits)
		res.tAccel -= tRollBackAccelPhase
		res.tDeccel -= tEarlyStopDeccelPhase
		return res

	@classmethod
	def computeFromRest(cls, S, limits: Limits) -> "PositiveSpeedPositionChangePlan":
		tAccel = sqrt(2 * S / ((1 + limits.accel / limits.deccel) * limits.accel))
		tDeccel = limits.accel / limits.deccel * tAccel

		maxSpeedDuringAccel = tAccel * limits.accel

		if maxSpeedDuringAccel > limits.speed:
			tAccel = limits.speed / limits.accel
			tDeccel = limits.speed / limits.deccel
			steadyPath = S - pathAccelDeccel(0, limits.accel, tAccel, limits.deccel, tDeccel)
			tSteady = steadyPath / limits.speed
		else:
			tSteady = 0
			steadyPath = 0

		return cls(
			tAccel=tAccel,
			tSteady=tSteady,
			tDeccel=tDeccel,
		)

	computeFromRest.__doc__ = (
		compute.__doc__
		+ """from the following state:
	speed = 0
	currentPosition = 0
	targetPosition = S
	"""
	)


@dataclass
class ArbitraryPositionChangePlan:
	"""Position change in picewise fashion. NOT QUANTIZED TO STEPS!
	1. deccelerate from initial speed to zero speed: `tPreDeccel` units of time are needed to deccelarate with maximum decceleration. Skip if 0.
	2. Execute the plan from `plan`, multiply the accelerations by `sign`
	3. accelerate from zero speed to final speed: `tPostAccel` units of time are needed to accelarate with maximum acceleration. Skip if 0.
	"""

	sign: int
	plan: PositiveSpeedPositionChangePlan
	tPreDeccel: float = 0
	tPostAccel: float = 0

	@property
	def t0(self):
		return self.tPreDeccel

	@property
	def t3(self):
		return self.t0 + self.plan.tTotal

	@property
	def tTotal(self):
		return self.t3 + self.tPostAccel

	def accel(self, t: float, limits: Limits) -> float:
		"""A piecewise acceleration function"""
		if t <= 0:
			return 0

		if t <= self.t0:
			return self.preDeccelAccelertion(limits)

		if t <= self.t3:
			return self.mainPartAccel(t, limits)

		if t > self.tTotal:
			return 0

		return self.postAccelAccelertion(limits)

	def preDeccelAccelertion(self, limits: Limits):
		return self.sign * limits.deccel

	def mainPartAccel(self, t: float, limits: Limits) -> float:
		return self.sign * self.plan.accel(t - self.t0, limits)

	def postAccelAccelertion(self, limits: Limits):
		return -self.sign * limits.accel

	def preDeccelPositionDelta(self, limits: Limits, initial: State) -> float:
		return initial.position + pathAccel(initial.speed, limits.deccel, self.tPreDeccel)

	def postAccelPositionDelta(self, limits: Limits) -> float:
		return pathAccel(0, -limits.accel, self.tPostAccel)

	def mainPhaseInitialSpeed(self, initial: State) -> float:
		return initial.speed if not self.tPreDeccel else 0

	def position(self, limits: Limits, initial: State) -> float:
		return self.preDeccelPositionDelta(limits, initial) + self.plan.position(limits, self.mainPhaseInitialSpeed(initial)) + self.postAccelPositionDelta(limits)

	def maxSpeed(self, limits: Limits, initial: State) -> float:
		return self.plan.maxSpeed(limits, self.mainPhaseInitialSpeed(initial))

	def finalSpeed(self, limits: Limits, initial: State):
		return self.plan.finalSpeed(limits, self.mainPhaseInitialSpeed(initial)) + self.tPostAccel * self.postAccelAccelertion(limits)

	@classmethod
	def compute(cls, initial: State, final: State, limits: Limits) -> "ArbitraryPositionChangePlan":
		"""ToDO: QUANTIZE!!!"""

		movementVector = final.position - initial.position
		movVectorSign = sign(movementVector)
		initialSpeedSign = sign(initial.speed)
		targetSpeedSign = sign(final.speed)
		initial.speedAndMovVectorHaveTheSameSign = movVectorSign == initialSpeedSign or initial.speed == 0 or movementVector == 0
		targetSpeedAndMovVectorHaveTheSameSign = movVectorSign == targetSpeedSign or final.speed == 0 or movementVector == 0
		initialSpeed = initial.speed
		finalSpeed = final.speed

		tRollBack = 0
		if not initial.speedAndMovVectorHaveTheSameSign:
			# decelelrate to zero first
			tPreDeccel = abs(initialSpeed) / limits.deccel
			deccelPath = initialSpeedSign * pathAccel(0, limits.deccel, tPreDeccel)
			movementVector -= deccelPath
			initialSpeed = 0
		else:
			tPreDeccel = 0

		if not targetSpeedAndMovVectorHaveTheSameSign:
			# accelerate from zero after
			tPostAccel = abs(finalSpeed) / limits.accel
			accelPath = targetSpeedSign * pathAccel(0, limits.accel, tPostAccel)
			movementVector -= accelPath
			finalSpeed = 0
		else:
			tPostAccel = 0

		S = movementVector * movVectorSign  # abs
		plan = PositiveSpeedPositionChangePlan.compute(S, limits, initialSpeed, finalSpeed)

		return cls(sign=movVectorSign, plan=plan, tPreDeccel=tPreDeccel, tPostAccel=tPostAccel)
