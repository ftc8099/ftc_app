package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle

/**
 * This class provides the basic functionality of a Kiwi drive using [KiwiKinematics].
 *
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kStatic additive constant feedforward
 * We're just going to ignore what these are supposed to be and say:
 * @param trackWidth distance between center of robot and wheels
 */
abstract class KiwiDrive constructor(
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        private val trackWidth: Double
) : Drive() {

    /**
     * Default localizer for kiwi drives based on the drive encoder positions, module orientations, and (optionally) a
     * heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class KiwiLocalizer @JvmOverloads constructor(
            private val drive: KiwiDrive,
            private val useExternalHeading: Boolean = true
    ) : Localizer {
        override var poseEstimate: Pose2d =
                Pose2d()
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                drive.externalHeading = value.heading
                field = value
            }
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val moduleOrientations = drive.getModuleOrientations()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = KiwiKinematics.wheelToRobotVelocities(
                        wheelDeltas, moduleOrientations, drive.trackWidth)
                val finalHeadingDelta = if (useExternalHeading)
                    Angle.norm(extHeading - lastExtHeading)
                else
                    robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate,
                        Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                )
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = KiwiLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = KiwiKinematics.robotToWheelVelocities(
                driveSignal.vel, trackWidth)
        val accelerations = KiwiKinematics.robotToWheelAccelerations(
                driveSignal.vel, driveSignal.accel, trackWidth)
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
        val orientations = KiwiKinematics.robotToModuleOrientations(
                driveSignal.vel, trackWidth)
        setMotorPowers(powers[0], powers[1], powers[2])
        setModuleOrientations(orientations[0], orientations[1], orientations[2])
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(drivePower, trackWidth)
        val orientations = KiwiKinematics.robotToModuleOrientations(drivePower, trackWidth)
        setMotorPowers(powers[0], powers[1], powers[2])
        setModuleOrientations(orientations[0], orientations[1], orientations[2])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(back: Double, right: Double, left: Double)

    /**
     * Sets the module orientations. All values are in radians.
     */
    abstract fun setModuleOrientations(back: Double, right: Double, left: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the current module orientations in radians. Orientations should exactly match the order in
     * [setModuleOrientations].
     */
    abstract fun getModuleOrientations(): List<Double>
}
