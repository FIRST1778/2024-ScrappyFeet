package org.chillout1778.subsystems

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj2.command.Subsystem

object Swerve: Subsystem {
    val modules = arrayOf(
        SwerveModule(1,5,9, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive),
        SwerveModule(2, 6, 10, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive),
        SwerveModule(3, 7, 11, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive),
        SwerveModule(4, 8, 12, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive)
    )
    val swerveKinematics = SwerveDriveKinematics(
        Translation2d(1.0, 1.0),
        Translation2d(1.0,-1.0),
        Translation2d(-1.0,1.0),
        Translation2d(-1.0,-1.0))
    fun driveRobotRelative(x: Double, y: Double, rotation: Double) {
        val chassisSpeeds = ChassisSpeeds(x,y,rotation)
        val moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds)
        for (i in modules.indices) {
            modules[i].drive(angle = moduleStates[i].angle.radians,
                driveVelocity = moduleStates[i].speedMetersPerSecond)
        }
    }
}