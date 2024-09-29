package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Elevator

class ElevatorMoveCommand(private val setpointFunction: () -> Unit) : Command() {
    init{
        addRequirements(Elevator)
    }
    override fun initialize() {
        setpointFunction()
    }
    override fun isFinished() = Elevator.atSetpoint
}
