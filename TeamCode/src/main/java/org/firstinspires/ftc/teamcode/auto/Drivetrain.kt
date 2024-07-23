package org.firstinspires.ftc.teamcode.auto

import dev.frozenmilk.util.units.position.Pose2D

interface Drivetrain {
	val localizer: Localizer

	fun target(pose: Pose2D)

	fun move(transformation: Pose2D)

	fun update()
}