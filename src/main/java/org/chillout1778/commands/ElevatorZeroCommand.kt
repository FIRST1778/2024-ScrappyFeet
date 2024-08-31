package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Elevator

class ElevatorZeroCommand : Command(){
    init{
        addRequirements(Elevator)
    }
    override fun initialize() {
        Elevator.zero()
        Elevator.zeroed = true
        Elevator.resetController()
    }
    override fun isFinished(): Boolean = true
}