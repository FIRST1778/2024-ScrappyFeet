package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.FlywheelFastCommand
import org.chillout1778.commands.RollerSetCommand
import org.chillout1778.subsystems.Rollers

object Controls {
    val controller = CommandXboxController(0)

    init {
        controller.leftTrigger()// Shoot
            .whileTrue(FlywheelFastCommand())
        controller.rightTrigger()// Intake
            .whileTrue(RollerSetCommand(Rollers.State.SUCK))
        controller.b()// Spit
            .whileTrue(RollerSetCommand(Rollers.State.SPIT))
    }

}