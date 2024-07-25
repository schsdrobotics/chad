package org.firstinspires.ftc.teamcode.util

import java.lang.System

class PIDFController(private var kP: Double, private var kI: Double, private var kD: Double, private var kF: Double, private var setPoint: Double, private var measuredValue: Double) {
	private var minIntegral = -1.0
	private var maxIntegral = 1.0

	private var positionError = 0.0
//		get() = setPoint - measuredValue

	private var velocityError = 0.0
//		get() = (positionError - prevErrorVal) / period

	private var totalError = 0.0
	private var prevErrorVal = 0.0

	private var errorTolerance_p = 0.05
	private var errorTolerance_v = Double.POSITIVE_INFINITY

	private var lastTimeStamp = 0.0
	private var period = 0.0

	constructor(kP: Double, kI: Double, kD: Double, kF: Double) : this(kP, kI, kD, kF, 0.0, 0.0)

	init {
		positionError = setPoint - measuredValue
		reset()
	}

	fun reset() {
		totalError = 0.0
		prevErrorVal = 0.0
		lastTimeStamp = 0.0
	}

	fun setTolerance(positionTolerance: Double) { setTolerance(positionTolerance, Double.POSITIVE_INFINITY) }

	/**
	 * Sets the error which is considered tolerable for use with [.atSetPoint].
	 *
	 * @param positionTolerance Position error which is tolerable.
	 * @param velocityTolerance Velocity error which is tolerable.
	 */
	fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
		errorTolerance_p = positionTolerance
		errorTolerance_v = velocityTolerance
	}

	/**
	 * Returns the current setpoint of the PIDFController.
	 *
	 * @return The current setpoint.
	 */
	fun getSetPoint(): Double {
		return setPoint
	}

	/**
	 * Sets the setpoint for the PIDFController
	 *
	 * @param sp The desired setpoint.
	 */
	fun setSetPoint(sp: Double) {
		setPoint = sp
		positionError = setPoint - measuredValue
		velocityError = (positionError - prevErrorVal) / period
	}

	fun atSetPoint() = (Math.abs(positionError) < errorTolerance_p && Math.abs(velocityError) < errorTolerance_v)

	fun getCoefficients() = listOf(kP, kI, kD, kF)

	/**
	 * @return the positional error e(t)
	 */
	fun getPositionError() = positionError

	/**
	 * @return the tolerances of the controller
	 */
	fun getTolerance() = listOf(errorTolerance_p, errorTolerance_v)

	/**
	 * @return the velocity error e'(t)
	 */
	fun getVelocityError() = velocityError

	/**
	 * Calculates the next output of the PIDF controller.
	 *
	 * @return the next output using the current measured value via
	 * [.calculate].
	 */
	fun calculate() = calculate(measuredValue)

	/**
	 * Calculates the next output of the PIDF controller.
	 *
	 * @param pv The given measured value.
	 * @param sp The given setpoint.
	 * @return the next output using the given measurd value via
	 * [.calculate].
	 */
	fun calculate(pv: Double, sp: Double): Double {
		// set the setpoint to the provided value
		setSetPoint(sp)
		return calculate(pv)
	}

	/**
	 * Calculates the control value, u(t).
	 *
	 * @param pv The current measurement of the process variable.
	 * @return the value produced by u(t).
	 */
	fun calculate(pv: Double): Double {
		prevErrorVal = positionError
		val currentTimeStamp = System.nanoTime().toDouble() / 1e9

		if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
		period = currentTimeStamp - lastTimeStamp
		lastTimeStamp = currentTimeStamp

		if (measuredValue == pv) {
			positionError = setPoint - measuredValue
		} else {
			positionError = setPoint - pv
			measuredValue = pv
		}
		velocityError =
			if (Math.abs(period) > 1E-6) (positionError - prevErrorVal) / period
			else 0.0

		/*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
		totalError += period * (setPoint - measuredValue)
		totalError = if (totalError < minIntegral) minIntegral else Math.min(maxIntegral, totalError)

		// returns u(t)
		return kP * positionError + kI * totalError + kD * velocityError + kF * setPoint
	}

	fun setPIDF(kp: Double, ki: Double, kd: Double, kf: Double) {
		kP = kp
		kI = ki
		kD = kd
		kF = kf
	}

	fun setIntegrationBounds(integralMin: Double, integralMax: Double) {
		minIntegral = integralMin
		maxIntegral = integralMax
	}

	// used to clear kI gains
	fun clearTotalError() { totalError = 0.0 }

	fun setP(kp: Double) { kP = kp }

	fun setI(ki: Double) { kI = ki }

	fun setD(kd: Double) { kD = kd }

	fun setF(kf: Double) { kF = kf }

	fun getP() = kP

	fun getI() = kI

	fun getD() = kD

	fun getF() = kF

	fun getPeriod() = period
}