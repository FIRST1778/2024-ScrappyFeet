package org.chillout1778.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.Utils
import org.chillout1778.subsystems.Swerve
import kotlin.math.sign

class TeleopGasPedalDriveCommand(val driver: CommandXboxController): Command() {
    fun getDriveSpeed() : Double {
        // driver.leftTriggerAxis is a value between 0 and 1 (is 0 unpressed?).
        //
        // leftTriggerAxis == 0?   getDriveSpeed() returns 1, aka full speed
        // leftTriggerAxis == 1?   getDriveSpeed() returns 0.25, aka quarter speed
        return 1 - (0.75 * driver.leftTriggerAxis)
    }

    fun squareWithSign(n: Double): Double {
        return n*n*sign(n)
    }

    fun getDriveX() : Double {
        return Utils.deadZone(
            squareWithSign(driver.leftX * Swerve.Constants.MAX_SPEED * getDriveSpeed()),
            0.1
        )
    }

    fun getDriveY() : Double {
        return Utils.deadZone(
            squareWithSign(driver.leftY * Swerve.Constants.MAX_SPEED * getDriveSpeed()),
            0.1
        )
    }

    fun getDriveR() : Double {
        return Utils.deadZone(
            squareWithSign(driver.rightX * Swerve.Constants.MAX_SPEED * getDriveSpeed()),
            0.1
        )
    }

    override fun execute() {
        Swerve.driveFieldRelative(ChassisSpeeds(
            getDriveX(), getDriveY(), getDriveR()
        ))
    }
}
