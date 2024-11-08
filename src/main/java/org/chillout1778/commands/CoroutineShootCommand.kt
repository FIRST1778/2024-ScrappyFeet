package org.chillout1778.commands

import kotlin.coroutines.cancellation.CancellationException

class ElevatorShootCommand: CoroutineCommand(Elevator) {
    override suspend fun runRoutine() {
        try {
            Elevator.up()
            wait { Elevator.atSetpoint() && Controls.driver.haveDriveApproval() }
            Shooter.shoot()
            wait { !Shooter.hasNote() }
            Shooter.stopShooting()
        } catch (e: CancellationException) {
        } finally {
            Elevator.down()
        }
    }
}