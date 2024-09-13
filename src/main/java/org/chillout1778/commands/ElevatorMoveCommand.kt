package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Elevator

class ElevatorMoveCommand(private var requestedPosition: Double) : Command(){
    private var requestedState : Constants.Elevator.ElevatorState? = null
    constructor(state: Constants.Elevator.ElevatorState) : this(state.position){
        requestedState = state
    }

    init{
        addRequirements(Elevator)
    }

    private var climbed = false

    override fun initialize() {
        climbed = Elevator.currentState == Constants.Elevator.ElevatorState.CLIMB_DOWN
        if(!climbed)
            Elevator.setpoint = requestedPosition
        else if(requestedState == Constants.Elevator.ElevatorState.CLIMB_UP) // prevents the shooter from going down when in climbed position.
            Elevator.setpoint = requestedPosition
    }
    override fun isFinished() = Elevator.atSetpoint

    override fun end(interrupted: Boolean) {
        if(!climbed || requestedState == Constants.Elevator.ElevatorState.CLIMB_UP) Elevator.currentState = requestedState
    }
}