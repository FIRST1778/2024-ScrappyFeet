package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.*

object Controls {
    val operatorController = CommandXboxController(1)
    val driverController = CommandXboxController(0)
    val driverApproval get() = operatorController.hid.aButton || Robot.isAutonomous

    init{
        operatorController.rightTrigger()                   //Intake
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)
                    .andThen(IntakeCommand())
            )
        operatorController.leftTrigger()                    //Shoot
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)
                    .andThen(ShooterShootCommand())
            )
        operatorController.leftBumper()                     //Amp
            .whileTrue(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.AMP)
                    .andThen(ShooterAmpCommand())
            )
            .onFalse(
                ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)
            )
        operatorController.b().whileTrue(ShooterSpitCommand())
        operatorController.pov(0)
            .onTrue(ElevatorMoveCommand(Constants.Elevator.ElevatorState.CLIMB_UP))
        operatorController.pov(180)
            .onTrue(ElevatorMoveCommand(Constants.Elevator.ElevatorState.CLIMB_DOWN))
    }
}