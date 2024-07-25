package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput


@Config
class AbsoluteAnalogEncoder @JvmOverloads constructor(val encoder: AnalogInput, private val analogRange: Double = DEFAULT_RANGE) {
	private var offset = 0.0
	var direction = false
		private set

	fun zero(off: Double): AbsoluteAnalogEncoder {
		offset = off
		return this
	}

	fun setInverted(invert: Boolean): AbsoluteAnalogEncoder {
		direction = invert
		return this
	}

	private var pastPosition = 1.0

	val currentPosition: Double
		get() {
			val pos: Double = Angle.norm((if (!direction) 1 - voltage / analogRange else voltage / analogRange) * Math.PI * 2 - offset)
			//checks for crazy values when the encoder is close to zero
			if (!VALUE_REJECTION || Math.abs(Angle.normDelta(pastPosition)) > 0.1 || Math.abs(Angle.normDelta(pos)) < 1) pastPosition = pos
			return pastPosition
		}
	val voltage: Double
		get() = encoder.voltage

	companion object {
		var DEFAULT_RANGE = 3.3
		var VALUE_REJECTION = false
	}
}