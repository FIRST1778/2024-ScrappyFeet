package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.FlywheelFastCommand
import org.chillout1778.commands.RollerSetCommand
import org.chillout1778.subsystems.Rollers
import org.chillout1778.subsystems.Swerve

object Controls {
    val driver = CommandXboxController(0)

    fun getDriveSpeed() : Double {
        return 1 - (0.75 * driver.leftTriggerAxis)
    }

    fun getDriveX() : Double {
        return Utils.deadZone(Math.pow(driver.leftX * Swerve.Constants.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }

    fun getDriveY() : Double {
        return Utils.deadZone(Math.pow(driver.leftY * Swerve.Constants.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }

    fun getDriveR() : Double {
        return Utils.deadZone(Math.pow(driver.rightX * Swerve.Constants.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }

    init {
        driver.leftTrigger()// Shoot
            .whileTrue(FlywheelFastCommand())
        driver.rightTrigger()// Intake
            .whileTrue(RollerSetCommand(Rollers.State.SUCK))
        driver.b()// Spit
            .whileTrue(RollerSetCommand(Rollers.State.SPIT))
    }

}