package org.chillout1778.subsystems

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.Controls
import org.chillout1778.Robot

object Swerve: Subsystem {
    object Constants {
        val MAX_VELOCITY = 1.0
        val MAX_ANGULAR_VELOCITY = 1.0
    }

    private val robotAngle: Double
        get() = 0.0 // TODO

    private val modules = arrayOf(
        SwerveModule(
            driveMotorID = 1, turnMotorID = 5, canCoderID = 9,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            driveMotorID = 2, turnMotorID = 6, canCoderID = 10,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            driveMotorID = 3, turnMotorID = 7, canCoderID = 11,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        ),
        SwerveModule(
            driveMotorID = 4, turnMotorID = 8, canCoderID = 12,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.CounterClockwise_Positive,
            canCoderOffsetDegrees = 0.0,
        )
    )

    private val swerveKinematics = SwerveDriveKinematics(
        Translation2d(1.0, 1.0),
        Translation2d(1.0,-1.0),
        Translation2d(-1.0,1.0),
        Translation2d(-1.0,-1.0),
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
        val moduleStates = swerveKinematics.toSwerveModuleStates(discreteSpeeds)
        for ((mod, state) in modules.zip(moduleStates)) {
            mod.driveState(state)
        }
    }
}
