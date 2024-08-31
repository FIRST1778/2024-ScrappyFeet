package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Elevator

class ElevatorMoveCommand(private var requestedPosition: Double) : Command(){
    constructor(state: Constants.Elevator.ElevatorState) : this(state.position)

    init{
        addRequirements(Elevator)
    }

    override fun initialize() {
        Elevator.setpoint = requestedPosition
    }
    override fun isFinished() = Elevator.atSetpoint
}