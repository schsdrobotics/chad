package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import dev.frozenmilk.dairy.calcified.Calcified
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.util.units.angle.Angle
import dev.frozenmilk.util.units.position.Pose2D
import dev.frozenmilk.util.units.position.Vector2D
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.auto.OTOSConfig.angularScalar
import org.firstinspires.ftc.teamcode.auto.OTOSConfig.calibrationSamples
import org.firstinspires.ftc.teamcode.auto.OTOSConfig.linearScalar
import org.firstinspires.ftc.teamcode.util.Translations.calcify
import org.firstinspires.ftc.teamcode.util.Translations.sparkify

class CalcifiedOTOS : Localizer {
	private val internal by OpModeLazyCell {
		Calcified.controlHub.unsafeGet(SparkFunOTOS::class.java, 0)
	}

	override fun init() {
		internal.linearUnit = DistanceUnit.INCH
		internal.angularUnit = AngleUnit.DEGREES

		internal.offset = Pose2D().sparkify()

		internal.setLinearScalar(linearScalar)
		internal.setAngularScalar(angularScalar)

		internal.resetTracking()

		internal.position = Pose2D().sparkify()

		internal.calibrateImu(calibrationSamples, false)
	}

	override fun set(pose: Pose2D) {
		internal.position = pose.sparkify()
	}

	override fun reset() {
		internal.resetTracking()
	}

	override fun position() = internal.position.calcify()
	override fun velocity() = internal.velocity.calcify()
	override fun acceleration() = internal.acceleration.calcify()
}

@Config
object OTOSConfig {
	@JvmField
	val linearScalar = 1.0

	@JvmField
	val angularScalar = 1.0

	@JvmField
	val calibrationSamples = 255
}