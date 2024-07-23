package org.firstinspires.ftc.teamcode.auto

import dev.frozenmilk.util.units.position.Pose2D

interface Localizer {
	fun init()
	fun reset()

	fun set(pose: Pose2D)

	fun position(): Pose2D
	fun velocity(): Pose2D
	fun acceleration(): Pose2D

}