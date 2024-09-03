package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.ElevatorMoveCommand
import org.chillout1778.commands.IntakeCommand
import org.chillout1778.commands.ShooterShootCommand
import org.chillout1778.commands.ShooterAmpCommand

object Controls {
    val driveController = CommandXboxController(0)
    val driverApproval get() = driveController.hid.aButton || Robot.isAutonomous

    init{
        driveController.rightTrigger()                   //Intake
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)
                    .andThen(IntakeCommand())
            )
        driveController.leftTrigger()                    //Shoot
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)
                    .andThen(ShooterShootCommand())
            )
        driveController.leftBumper()                     //Amp
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.AMP)
                    .andThen(ShooterAmpCommand())
            )
    }
}