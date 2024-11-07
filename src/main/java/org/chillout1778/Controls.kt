package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.FlywheelFastCommand
import org.chillout1778.commands.RollerSetCommand
import org.chillout1778.subsystems.Rollers

object Controls {
    val driver = CommandXboxController(0)

    init {
        driver.leftTrigger()// Shoot
            .whileTrue(FlywheelFastCommand())
        driver.rightTrigger()// Intake
            .whileTrue(RollerSetCommand(Rollers.State.SUCK))
        driver.b()// Spit
            .whileTrue(RollerSetCommand(Rollers.State.SPIT))
    }

}