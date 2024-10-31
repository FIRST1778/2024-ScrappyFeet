package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.FlywheelFastCommand
import org.chillout1778.commands.RollerSetCommand
import org.chillout1778.subsystems.Rollers
import org.chillout1778.commands.SaveSwerveModuleOffsetsCommand

object Controls {
    val driver = CommandXboxController(0)

    fun getDriveSpeed() : Double {
        val triggerSpeed = 1 - (0.75 * driver.leftTriggerAxis)
        val joystickSpeed = Math.hypot(driver.leftX, driver.leftY)
        return joystickSpeed * triggerSpeed
    }

    fun getDriveX() : Double {
        Utils.deadZone(Math.pow(return driver.leftX * Constants.Swerve.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }

    fun getDriveY() : Double {
        Utils.deadZone(Math.pow(return driver.leftY * Constants.Swerve.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }

    fun getDriveR() : Double {
        Utils.deadZone(Math.pow(return driver.rightX * Constants.Swerve.MAX_SPEED * getDriveSpeed(), 2.0), 0.1)
    }



    init {
        driver.leftTrigger()// Shoot
            .whileTrue(FlywheelFastCommand())
        driver.rightTrigger()// Intake
            .whileTrue(RollerSetCommand(Rollers.State.SUCK))
        driver.b()// Spit
            .whileTrue(RollerSetCommand(Rollers.State.SPIT))
        driver.x()
            .whileTrue(SaveSwerveModuleOffsetsCommand())
    }

}
