package org.firstinspires.ftc.teamcode.util

import dev.frozenmilk.util.units.angle.Angle
import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.position.Pose2D

object PoseUtils {
	val Pose2D.x: Distance
		get() = this.vector2D.x

	val Pose2D.y: Distance
		get() = this.vector2D.y

	val Pose2D.θ: Angle
		get() = this.heading
}