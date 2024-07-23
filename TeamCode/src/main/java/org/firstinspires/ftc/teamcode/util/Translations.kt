package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import dev.frozenmilk.util.units.angle.wrappedDeg
import dev.frozenmilk.util.units.distance.inches
import dev.frozenmilk.util.units.position.Pose2D
import dev.frozenmilk.util.units.position.Vector2D

typealias SparkFunPose2D = SparkFunOTOS.Pose2D
typealias CalcifiedPose2D = Pose2D

object Translations {
	fun SparkFunPose2D.calcify() =
		CalcifiedPose2D(Vector2D(this.x.inches, this.y.inches), this.h.wrappedDeg)

	fun CalcifiedPose2D.sparkify() =
		SparkFunPose2D(
			this.vector2D.x.intoInches().value,
			this.vector2D.y.intoInches().value,
			this.heading.intoDegrees().value,
		)
}