package org.firstinspires.ftc.teamcode.auto

import EnhancedPose2DSupplier
import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.calcified.Calcified
import dev.frozenmilk.dairy.calcified.hardware.controller.ComplexController
import dev.frozenmilk.dairy.calcified.hardware.controller.compiler.Pose2DControllerCompiler
import dev.frozenmilk.dairy.calcified.hardware.controller.compiler.Vector2DControllerCompiler
import dev.frozenmilk.dairy.calcified.hardware.controller.implementation.Pose2DController
import dev.frozenmilk.dairy.calcified.hardware.controller.implementation.Vector2DController
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents
import dev.frozenmilk.dairy.core.util.supplier.numeric.unit.EnhancedUnitSupplier
import dev.frozenmilk.util.units.angle.deg
import dev.frozenmilk.util.units.angle.wrappedDeg
import dev.frozenmilk.util.units.distance.inches
import dev.frozenmilk.util.units.position.Pose2D
import dev.frozenmilk.util.units.position.Vector2D
import org.checkerframework.checker.units.qual.Angle
import org.firstinspires.ftc.teamcode.auto.MecanumConfig.Ports

class MecanumDrive : Drivetrain {
	override val localizer = CalcifiedOTOS()

	val fr by OpModeLazyCell { Calcified.controlHub.getMotor(Ports.fr) }
	val fl by OpModeLazyCell { Calcified.controlHub.getMotor(Ports.fl) }
	val br by OpModeLazyCell { Calcified.controlHub.getMotor(Ports.br) }
	val bl by OpModeLazyCell { Calcified.controlHub.getMotor(Ports.bl) }
	val motors by OpModeLazyCell { listOf(fr, fl, br, bl) }

	val p2p = P2P(this)
	var target = Pose2D()

	override fun target(pose: Pose2D) {
		target = pose
		p2p.target(pose)
	}

	override fun move(transformation: Pose2D) {
		val normalized = transformation.vector2D.normalise(1.inches)

		val speed = normalized.magnitude.value
		val heading = normalized.theta.intoWrapping()

		val rotation = transformation.heading

		listOf(fl, br).forEach { it.power = (heading + 45.wrappedDeg).sin * speed }
		listOf(fr, bl).forEach { it.power = (heading - 45.wrappedDeg).sin * speed }
	}

	override fun update() {
		p2p.update()

		val x = p2p.x.output
		val y = p2p.y.output
		val θ = p2p.θ.output

		move(Pose2D(Vector2D(x, y), θ))
	}
}

@Config
object MecanumConfig {
	object Ports {
		const val fr = 0
		const val fl = 1
		const val br = 2
		const val bl = 3
	}
}