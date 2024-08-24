package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Drivetrain

class DriveCommand : Command(){
    init{
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.drive(Controls.driveController.hid.leftY,Controls.driveController.hid.rightX)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.drive(0.0, 0.0)
    }
}