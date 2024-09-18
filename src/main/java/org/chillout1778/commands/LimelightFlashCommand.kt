package org.chillout1778.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.lib.LimelightHelpers

class LimelightFlashCommand: Command() {
    val timer = Timer()
    override fun initialize() {
        LimelightHelpers.setLEDMode_ForceBlink("limelight")
        timer.reset()
        timer.start()
    }
    override fun isFinished() = timer.get() > .5
    override fun end(interrupted: Boolean) {
        LimelightHelpers.setLEDMode_ForceOff("limelight")
    }
}