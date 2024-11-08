package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.coroutines.cancellation.CancellationException

class CoroutineShootCommand: CoroutineCommand(Elevator, Shooter) {
    // Not a real command

    override suspend fun runRoutine() {
        try {
            Elevator.up()
            wait { Elevator.atSetpoint() && driverApproval() }
            Shooter.shoot()
            wait { !Shooter.hasNote() }
            Shooter.stopShooting()
        } catch (e: CancellationException) {
        } finally {
            Elevator.down()
        }
    }

    object Elevator: Subsystem {
        fun up() {}
        fun down() {}
        fun atSetpoint(): Boolean = false
    }
    object Shooter: Subsystem {
        fun hasNote(): Boolean = false
        fun shoot() {}
        fun stopShooting() {}
    }
    fun driverApproval(): Boolean = false
}