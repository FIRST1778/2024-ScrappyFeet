package org.chillout1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.StraightForwardCommand
import org.chillout1778.commands.WilliamTestCommand
import org.chillout1778.subsystems.Swerve

object Controls {
    val droneController = CommandGenericHID(0)

    data class DriveInputs(
        val forward: Double,
        val left: Double,
        val rotation: Double,
    )
    fun getDmytroControllerDriveInputs(): DriveInputs {
        return DriveInputs(
            forward = -droneController.getRawAxis(1),
            left = -droneController.getRawAxis(0),
            rotation = -droneController.getRawAxis(2)
        )
    }

    fun getDroneControllerDriveInputs(): DriveInputs {
        return DriveInputs(
            forward = droneController.getRawAxis(2),
            left = -droneController.getRawAxis(3),
            rotation = -droneController.getRawAxis(0)
        )

    }

    fun bindTriggers() {
        // No Commands to bind yet.
//        val cmd = StraightForwardCommand()
//        Shuffleboard.getTab("Commands").add("sysid cmd", cmd).withPosition(1, 1).withSize(1, 1)
//        droneController.axisGreaterThan(4, 0.5).whileTrue(
//            InstantCommand({
//            Swerve.robotAngle = 0.0
//            Swerve.odometry.resetPosition(Rotation2d(), Swerve.getAllModulePositions(), Pose2d())}))
//            WilliamTestCommand()
//            cmd
//        )

    }

     private val operatorController = CommandXboxController(1)
     private fun getOperatorControllerDriveInputs(): DriveInputs {
         return DriveInputs(
             forward = operatorController.leftY,
             left = -operatorController.leftX,
             rotation = operatorController.rightX,
         )
     }
}
