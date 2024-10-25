package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Rollers

class RollerSetCommand(private val state: Rollers.State) : Command() {
    init {
        addRequirements(Rollers)
    }
    override fun initialize() {
        Rollers.setState(state)
    }
    override fun end(interrupted: Boolean) {
        if(interrupted) Rollers.setState(Rollers.State.STOP)
    }
}