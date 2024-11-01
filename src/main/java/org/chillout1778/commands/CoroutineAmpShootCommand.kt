package org.chillout1778.commands

import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Shooter

class CoroutineAmpShootCommand: CoroutineCommand(Elevator, Shooter) {
    override suspend fun runRoutine() {
        try {
            Elevator.setpoint = Constants.Elevator.ElevatorState.AMP.position
            wait { Elevator.atSetpoint && Controls.driverApproval }
            Shooter.spit()
            waitForever()
        } finally {
            Elevator.setpoint = Constants.Elevator.ElevatorState.STORED.position
        }
    }
}