package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Swerve drive kinematic equations. All wheel positions and velocities are given starting with front left and
 * proceeding counter-clockwise (i.e., front left, rear left, rear right, front right). Robot poses are specified in a
 * coordinate system with positive x pointing forward, positive y pointing left, and positive heading measured
 * counter-clockwise from the x-axis.
 */
object KiwiKinematics {

    /**
     * Computes the wheel velocity vectors corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * We're just going to ignore what these are supposed to be and say:
     * @param trackWidth distance between center of robot and wheels
     */
    @JvmStatic
    fun robotToModuleVelocityVectors(
            robotVel: Pose2d,
            trackWidth: Double
    ): List<Vector2d> {
        val x = trackWidth / 2
        val y = trackWidth * sqrt(3.0) /2

        val vx = robotVel.x
        val vy = robotVel.y
        val omega = robotVel.heading

        return listOf(
                Vector2d(vx + 0        , vy - omega * trackWidth),
                Vector2d(vx + omega * y, vy + omega * x),
                Vector2d(vx - omega * y, vy + omega * x)
        )
    }

    /**
     * Computes the wheel velocities corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * We're just going to ignore what these are supposed to be and say:
     * @param trackWidth distance between center of robot and wheels
     */
    @JvmStatic
    fun robotToWheelVelocities(robotVel: Pose2d, trackWidth: Double) =
            robotToModuleVelocityVectors(
                    robotVel,
                    trackWidth
            ).map(Vector2d::norm)

    /**
     * Computes the module orientations (in radians) corresponding to [robotVel] given the provided
     * [trackWidth].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    @JvmStatic
    fun robotToModuleOrientations(robotVel: Pose2d, trackWidth: Double) =
            robotToModuleVelocityVectors(
                    robotVel,
                    trackWidth
            ).map(Vector2d::angle)

    /**
     * Computes the acceleration vectors corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     */
    @JvmStatic
    fun robotToModuleAccelerationVectors(
            robotAccel: Pose2d,
            trackWidth: Double
    ): List<Vector2d> {
        val x = trackWidth / 2
        val y = trackWidth * sqrt(3.0) /2

        val ax = robotAccel.x
        val ay = robotAccel.y
        val alpha = robotAccel.heading

        return listOf(
                Vector2d(ax + 0        , ay - alpha * trackWidth),
                Vector2d(ax + alpha * y, ay + alpha * x),
                Vector2d(ax - alpha * y, ay + alpha * x)
        )
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * We're just going to ignore what these are supposed to be and say:
     * @param trackWidth distance between center of robot and wheels
     */
    @JvmStatic
    fun robotToWheelAccelerations(
            robotVel: Pose2d,
            robotAccel: Pose2d,
            trackWidth: Double
    ) =
            robotToModuleVelocityVectors(
                    robotVel,
                    trackWidth
            ).zip(
                    robotToModuleAccelerationVectors(
                            robotAccel,
                            trackWidth
                    )
            )
                    .map { (vel, accel) ->
                        (vel.x * accel.x + vel.y * accel.y) / vel.norm()
                    }

    /**
     * Computes the module angular velocities corresponding to [robotAccel] given the provided [trackWidth]
     * and [wheelBase].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * We're just going to ignore what these are supposed to be and say:
     * @param trackWidth distance between center of robot and wheels
     */
    @JvmStatic
    fun robotToModuleAngularVelocities(
            robotVel: Pose2d,
            robotAccel: Pose2d,
            trackWidth: Double
    ) =
            robotToModuleVelocityVectors(
                    robotVel,
                    trackWidth
            ).zip(
                    robotToModuleAccelerationVectors(
                            robotAccel,
                            trackWidth
                    )
            ).map { (vel, accel) ->
                (vel.x * accel.y - vel.y * accel.x) / (vel.x * vel.x + vel.y * vel.y)
            }

    /**
     * Computes the robot velocities corresponding to [wheelVelocities], [moduleOrientations], and the drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param moduleOrientations wheel orientations (in radians)
     * We're just going to ignore what these are supposed to be and say:
     * @param trackWidth distance between center of robot and wheels
     */
    @JvmStatic
    fun wheelToRobotVelocities(
            wheelVelocities: List<Double>,
            moduleOrientations: List<Double>,
            trackWidth: Double
    ): Pose2d {
        val x = trackWidth / 2
        val y = trackWidth * sqrt(3.0) /2

        val vectors = wheelVelocities
                .zip(moduleOrientations)
                .map { (vel, orientation) ->
                    Vector2d(
                            vel * cos(orientation),
                            vel * sin(orientation)
                    )
                }

        val vx = vectors.sumByDouble { it.x } / 3
        val vy = vectors.sumByDouble { it.y } / 3
        val (back, right, left) = vectors
        val omega = (
                y * (right.x - left.x) +
                        x * (left.y + right.y) - trackWidth * back.y
                ) / (3 * (x * x + y * y))

        return Pose2d(vx, vy, omega)
    }
}
