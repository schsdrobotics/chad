package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder
import org.firstinspires.ftc.teamcode.util.PIDFController
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule(val motor: DcMotorEx, val servo: CRServo, val encoder: AbsoluteAnalogEncoder) {
	var P = 0.15
	var I = 0.0
	var D = 0.0
	var K_STATIC = 0.03

	var MAX_SERVO = 1.0
	var MAX_MOTOR = 1.0

	var MOTOR_FLIPPING = true

	var WHEEL_RADIUS = 1.4 // in

	var GEAR_RATIO = 1 / (3.5 * 1.5 * 2) // output (wheel) speed / input (motor) speed

	val TICKS_PER_REV = 28.0

	private val rotationController by lazy {
		PIDFController(P, I, D, 0.0)
	}

	var wheelFlipped = false
	private val target = 0.0
	private var position = 0.0

	constructor(
		hardwareMap: HardwareMap,
		mName: String,
		sName: String,
		eName: String
	) : this(
			hardwareMap[mName] as DcMotorEx,
			hardwareMap[sName] as CRServo,
			AbsoluteAnalogEncoder(hardwareMap[eName] as AnalogInput)
		)

	init {
		val motorConfigurationType = motor.motorType.clone()
		motorConfigurationType.achieveableMaxRPMFraction = MAX_MOTOR
		motor.motorType = motorConfigurationType
		motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

		(servo as CRServoImplEx).pwmRange = PwmRange(500.0, 2500.0, 5000.0)

		rotationController
		motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
	}

	fun read() { position = encoder.getCurrentPosition() }

	fun update() {
		rotationController.setPIDF(P, I, D, 0.0)
		var target: Double = getTargetRotation()
		val current: Double = getModuleRotation()
		var error = AngleUnit.normalizeRadians(target - current)

		if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
			target = AngleUnit.normalizeRadians(target - Math.PI)
			wheelFlipped = true
		} else {
			wheelFlipped = false
		}

		error = AngleUnit.normalizeRadians(target - current)
		var power = Range.clip(rotationController.calculate(0.0, error), -MAX_SERVO, MAX_SERVO)
		if (java.lang.Double.isNaN(power)) power = 0.0
		servo.power = (power + (if (abs(error) > 0.02) K_STATIC else 0.0)) * sign(power)
	}


}