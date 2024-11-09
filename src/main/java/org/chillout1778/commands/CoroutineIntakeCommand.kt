package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.coroutines.cancellation.CancellationException

class CoroutineIntakeCommand: CoroutineCommand(Rollers) {
    // Not a real command

    override suspend fun runRoutine() {
        try {
            Rollers.startIntaking()
            wait { Rollers.haveNote() }
            Rollers.reverseFlywheels()
            waitSeconds(0.1)
            Rollers.stopFlywheels()
        } catch (e: CancellationException) {
            println("${this::class.simpleName} got interrupted!")
        } finally {
            Rollers.stopIntaking()
            Rollers.stopFlywheels()
        }
    }

    // More fake components (see ElevatorShootCommand)
    object Rollers: Subsystem {
        fun startIntaking() {}
        fun stopIntaking() {}
        fun reverseFlywheels() {}
        fun stopFlywheels() {}
        fun haveNote() = false
    }
}
