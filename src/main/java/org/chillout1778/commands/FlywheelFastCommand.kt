package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Flywheels

class FlywheelFastCommand: Command() {
    // initialize()
    // execute()
    // isFinished(): Boolean
    // end(interrupted: Boolean)
    override fun initialize() {
        addRequirements(Flywheels)
        Flywheels.fast()
    }

    override fun isFinished(): Boolean {
        return Flywheels.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            Flywheels.off()
        }
    }
}