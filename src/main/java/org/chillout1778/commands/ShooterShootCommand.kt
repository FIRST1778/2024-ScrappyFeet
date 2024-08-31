package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Shooter

class ShooterShootCommand : Command(){
    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        Shooter.revFlywheels()
    }

    override fun execute() {
        if(Controls.driverApproval) Shooter.suck()
    }
}