package org.chillout1778

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.*
import org.chillout1778.subsystems.Elevator

object Controls {
    val operatorController = CommandXboxController(1)
    val driverController = CommandJoystick(0)
    val driverApproval get() = operatorController.hid.aButton || Robot.isAutonomous || driverController.getRawAxis(7) > .5

    init{
        operatorController.rightTrigger()                   //Intake
            .whileTrue(
                ElevatorMoveCommand(Elevator::down)
                    .andThen(IntakeCommand())
            )
        operatorController.leftTrigger()                    //Shoot
            .whileTrue(
                ElevatorMoveCommand(Elevator::down)
                    .andThen(ShooterShootCommand())
            )
        operatorController.leftBumper()                     //Amp
            .whileTrue(
                ElevatorMoveCommand(Elevator::up)
                    .andThen(ShooterAmpCommand())
            )
            .onFalse(
                ElevatorMoveCommand(Elevator::down)
            )
        operatorController.b().whileTrue(ShooterSpitCommand())
    }
}
