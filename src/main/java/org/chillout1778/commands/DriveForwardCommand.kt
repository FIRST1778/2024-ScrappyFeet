package org.chillout1778.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Drivetrain

class DriveForwardCommand(val driveSpeed : Double, val turnSpeed : Double, val duration : Double): Command() {
    init{
        addRequirements(Drivetrain)
    }
    val timer = Timer()
    override fun initialize() {
        timer.reset()
        timer.start()
        DriverStation.reportWarning("Driving", false)
    }

    override fun execute() {
        Drivetrain.drive(driveSpeed, turnSpeed)
    }

    override fun isFinished() = timer.get() >= duration

    override fun end(interrupted: Boolean) {
        timer.stop()
        timer.reset()
        Drivetrain.drive(0.0, 0.0)
        if(interrupted)
            DriverStation.reportWarning("Interrupted", false)
        else
            DriverStation.reportWarning("Ended", false)
    }
}