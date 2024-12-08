package org.chillout1778.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import java.util.function.Supplier
import org.chillout1778.Controls
import org.chillout1778.Controls.DriveInputs
import org.chillout1778.subsystems.Swerve
import org.chillout1778.Utils
import kotlin.math.*

class StraightForwardCommand(
    private var volts: Double = 6.0,
) : Command(), Sendable {
    init {
        addRequirements(Swerve)
    }

    var maxSpeed = 0.0

    override fun initialize() {
        maxSpeed = 0.0
    }

    override fun execute() {
        println("going straight forward at ${volts} volts")
        var curSpeed = Swerve.getOverallSpeed()
        if (curSpeed > maxSpeed) maxSpeed = curSpeed
        Swerve.driveFieldRelative(
            ChassisSpeeds(volts, 0.0, 0.0)
        )
    }

    override fun end(_interrupted: Boolean) {
        Swerve.driveFieldRelative(ChassisSpeeds()) // brake robot
        println("max speed ================ ${maxSpeed}")
    }

    override fun initSendable(builder: SendableBuilder) {
        builder!!
        builder.addDoubleProperty("voltage", {volts}, {volts = it})
    }
}
