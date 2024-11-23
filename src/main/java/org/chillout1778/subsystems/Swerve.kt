package org.chillout1778.subsystems

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.PI
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.Controls
import org.chillout1778.Robot

object Swerve: Subsystem {
    object Constants {
        // How fast the robot can move in a straight line (meters/sec).
        val MAX_VELOCITY = 1.0
        // How fast the robot can rotate (radians/sec).
        val MAX_ANGULAR_VELOCITY = 2*PI
        // How far the swerve modules are from (0,0).
        val XY_DISTANCE = Units.inchesToMeters(11.75)
    }

    // Read yaw from the gyro (radians, counterclockwise from forward).
    var robotAngle: Double
        get() = 0.0 // TODO
        set(n) {
            // Reset yaw to n.
        }

    // Data for all the modules.  They are combined in one big array
    // because that's the "smart" way to do it.  A lot of teams
    // duplicate everything: "frontLeftSwerveModuleDriveMotorId = 1".
    // I hate those variables, the whole point of classes is so we can
    // do better than that.       However...the positions of the modules
    // are listed below in the SwerveKinematics class...so do change that
    // if you change the order.
    //
    // The order shouldn't matter, but PathPlannerLib prefers FL, FR,
    // BL, BR order so we'll just use that.
    private val modules = arrayOf(
        SwerveModule(
            name = "Front Left",
            driveMotorID = 1, turnMotorID = 5, canCoderID = 9,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            name = "Front Right",
            driveMotorID = 2, turnMotorID = 6, canCoderID = 10,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            name = "Back Left",
            driveMotorID = 3, turnMotorID = 7, canCoderID = 11,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            name = "Back Right",
            driveMotorID = 4, turnMotorID = 8, canCoderID = 12,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        )
    )

    private val kinematics = SwerveDriveKinematics(
        Translation2d(1.0, 1.0).times(Constants.XY_DISTANCE),
        Translation2d(1.0, -1.0).times(Constants.XY_DISTANCE),
        Translation2d(-1.0, 1.0).times(Constants.XY_DISTANCE),
        Translation2d(-1.0, -1.0).times(Constants.XY_DISTANCE),
    )

    fun driveFieldRelative(speeds: ChassisSpeeds) {
        driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Rotation2d.fromRadians(robotAngle)
            )
        )
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.period)
        val moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        for ((mod, state) in modules.zip(moduleStates)) {
            mod.driveState(state)
        }
    }
}
