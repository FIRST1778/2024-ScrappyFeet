package org.chillout1778.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.Controls.driver
import org.chillout1778.Utils
import org.chillout1778.subsystems.Swerve

class TeleopGasPedalDriveCommand(driver: CommandXboxController): Command() {
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

    override fun execute() {
        Swerve.driveFieldRelative(ChassisSpeeds(
            getDriveX(), getDriveY(), getDriveR()
        ))
    }
}