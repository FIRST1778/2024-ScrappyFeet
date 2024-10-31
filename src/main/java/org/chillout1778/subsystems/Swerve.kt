package org.chillout1778.subsystems

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Subsystem
import java.io.File
import java.io.FileNotFoundException

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

    val swerveOffsetPath = "${System.getProperty("user.home")}/swerve-offsets.txt"

    fun initializeSwerveOffsets() {
        val text = try {
            File(swerveOffsetPath).readText()
        } catch (e: FileNotFoundException) {
            DriverStation.reportError(
                "COULD NOT READ SWERVE OFFSETS: $e", false
            )
            return
        }

        val offsets = text
            .split(Regex("\\s+"))
            .map(String::toDoubleOrNull)
            .filterNotNull()
        if (offsets.size < modules.size) {
            DriverStation.reportError(
                "COULD NOT READ SWERVE OFFSETS: needed ${modules.size}, got ${offsets.size}",
                false
            )
            return
        }

        for ((module, offset) in modules.zip(offsets)) {
            module.canCoderOffset = offset
        }
    }

    fun saveSwerveOffsets() {
        val offsets = modules.map(SwerveModule::turnPosition)
        File(swerveOffsetPath).writeText("${offsets.joinToString(" ")}\n")
        DriverStation.reportWarning("Saved swerve offsets", false)
    }
}
