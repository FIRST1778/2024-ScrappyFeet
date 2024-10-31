package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Swerve

class SaveSwerveModuleOffsetsCommand: Command() {
    var i: Int = 0
    override fun initialize() {
        i = 0
    }
    override fun execute() {
        if ((i == 0 || i == 3) && Controls.driver.hid.yButtonPressed)
            i++
        else if ((i == 1 || i == 2) && Controls.driver.hid.bButtonPressed)
            i++
        if (i == 4) {
            Swerve.saveSwerveOffsets()
            i = 0
        }
    }
}
