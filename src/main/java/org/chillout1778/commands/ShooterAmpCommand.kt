package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Shooter

class ShooterAmpCommand: Command() {
    init {
        addRequirements(Shooter)
    }

    override fun execute() {
        if(Controls.driverApproval){
            Shooter.spit()
        }
    }
    override fun end(interrupted: Boolean) {
        Shooter.stopRollers()
    }
}
