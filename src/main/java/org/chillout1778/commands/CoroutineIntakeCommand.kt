package org.chillout1778.commands

import org.chillout1778.subsystems.Rollers
import kotlin.coroutines.cancellation.CancellationException

class AsdfCommand: CoroutineCommand(Rollers) {
    override suspend fun runRoutine() {
        try {
            Rollers.startIntaking()
            wait { Rollers.haveNote() }
            Rollers.reverseFlywheels()
            waitSeconds(0.1)
            Rollers.stopFlywheels()
        } catch (e: CancellationException) {
            println("AsdfCommand got interrupted!")
        } finally {
            Rollers.stopIntaking()
            Rollers.stopFlywheels()
        }
    }
    // end of runRoutine
}
