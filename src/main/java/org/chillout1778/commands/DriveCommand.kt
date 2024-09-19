package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.sign

class DriveCommand : Command(){
    init{
        addRequirements(Drivetrain)
    }

    private fun square(n: Double) = n//*n* sign(n)
    private fun deadband(n: Double) = if(abs(n) < 0.05) 0.0 else n

    override fun execute() {
//        Drivetrain.drive(square(deadband(Controls.driverController.getRawAxis(2))), square(deadband(Controls.driverController.getRawAxis(0))))
        Drivetrain.drive(square(deadband(-Controls.operatorController.leftY)), square(deadband(-Controls.operatorController.rightX)))
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.drive(0.0, 0.0)
    }
}