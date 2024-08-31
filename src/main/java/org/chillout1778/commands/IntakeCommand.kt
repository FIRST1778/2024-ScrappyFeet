package org.chillout1778.commands

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Robot
import org.chillout1778.subsystems.CenteringWheels
import org.chillout1778.subsystems.CenteringWheels.passiveSpit
import org.chillout1778.subsystems.Shooter

class IntakeCommand: Command(){
    init {
        addRequirements(Shooter, CenteringWheels)
    }
    override fun initialize() {
        Shooter.suck()
        CenteringWheels.suck()
    }
    override fun isFinished() = Shooter.noteStored

    override fun end(interrupted: Boolean) {
        Shooter.stopRollers()
        passiveSpit()
    }
}