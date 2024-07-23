package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.calcified.hardware.controller.ComplexController
import dev.frozenmilk.dairy.calcified.hardware.controller.calculation.UnitDComponent
import dev.frozenmilk.dairy.calcified.hardware.controller.calculation.UnitIComponent
import dev.frozenmilk.dairy.calcified.hardware.controller.calculation.UnitPComponent
import dev.frozenmilk.dairy.calcified.hardware.controller.compiler.UnitControllerCompiler
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents.POSITION
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents.VELOCITY
import dev.frozenmilk.dairy.core.util.supplier.numeric.unit.EnhancedUnitSupplier
import dev.frozenmilk.util.units.angle.Angle
import dev.frozenmilk.util.units.angle.AngleUnit
import dev.frozenmilk.util.units.angle.deg
import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.distance.DistanceUnit
import dev.frozenmilk.util.units.distance.DistanceUnits.METER
import dev.frozenmilk.util.units.distance.DistanceUnits.MILLIMETER
import dev.frozenmilk.util.units.distance.feet
import dev.frozenmilk.util.units.distance.meters
import dev.frozenmilk.util.units.distance.mm
import dev.frozenmilk.util.units.position.Pose2D
import org.firstinspires.ftc.teamcode.util.PoseUtils.x
import org.firstinspires.ftc.teamcode.util.PoseUtils.y
import org.firstinspires.ftc.teamcode.util.PoseUtils.θ

class P2P(val drive: Drivetrain) {
	val x: ComplexController<Distance>
	val y: ComplexController<Distance>
	val θ: ComplexController<Angle>

	init {
		val xSupplier = EnhancedUnitSupplier({ drive.localizer.position().x })
		val xCompiler = UnitControllerCompiler<DistanceUnit, Distance>()
			.withSupplier(xSupplier)
			.append(UnitPComponent(0.5))
			.append(UnitDComponent(0.5))
			.append(UnitIComponent(0.5))

		x = xCompiler.compile(0.feet, POSITION, 3.mm)

		val ySupplier = EnhancedUnitSupplier({ drive.localizer.position().y })
		val yCompiler = UnitControllerCompiler<DistanceUnit, Distance>()
			.withSupplier(ySupplier)
			.append(UnitPComponent(0.5))
			.append(UnitDComponent(0.5))
			.append(UnitIComponent(0.5))

		y = yCompiler.compile(0.feet, POSITION, 3.mm)

		val θSupplier = EnhancedUnitSupplier({ drive.localizer.position().θ })
		val θCompiler = UnitControllerCompiler<AngleUnit, Angle>()
			.withSupplier(θSupplier)
			.append(UnitPComponent(0.5))
			.append(UnitDComponent(0.5))
			.append(UnitIComponent(0.5))

		θ = θCompiler.compile(0.deg, POSITION, 1.deg)
	}

	val controllers = listOf(x, y, θ)

	fun target(pose: Pose2D) {
		x.target = pose.x
		y.target = pose.y
		θ.target = pose.θ
	}

	fun update() = controllers.forEach { it.update() }
}

object P2PConfig {
	@Config
	object XPID {
		@JvmField
		val P = 0.0

		@JvmField
		val I = 0.0

		@JvmField
		val D = 0.0
	}

	@Config
	object YPID {
		@JvmField
		val P = 0.0

		@JvmField
		val I = 0.0

		@JvmField
		val D = 0.0
	}

	@Config
	object θPID {
		@JvmField
		val P = 0.0

		@JvmField
		val I = 0.0

		@JvmField
		val D = 0.0
	}
}