package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Shooter

class ShooterSpitCommand: Command() {
    init{
        addRequirements(Shooter)
    }
    override fun initialize() {
        Shooter.slowSpit()
    }

    override fun end(interrupted: Boolean) {
        Shooter.stopRollers()
    }
}