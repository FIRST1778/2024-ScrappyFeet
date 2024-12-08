package org.chillout1778.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import org.chillout1778.subsystems.Swerve
import kotlin.coroutines.cancellation.CancellationException

class WilliamTestCommand : CoroutineCommand(Swerve) {
    override suspend fun runRoutine() {
        try {
            val timer = Timer()
            timer.start()
            while (!timer.hasElapsed(1.0)) {
                Swerve.driveFieldRelative(ChassisSpeeds(0.1,0.0,0.0))
                yield()
            }
        } catch (e: CancellationException) {
            // command was canceled!

        } finally {
            Swerve.driveFieldRelative(ChassisSpeeds(0.0,0.0,0.0))
        }
    }
}
