package org.chillout1778

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.chillout1778.commands.TeleopDriveCommand

object Robot : TimedRobot() {
    override fun robotInit() {
        Controls
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
    }


    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    // These functions run the TeleopDriveCommand during teleop,
    // basically just scheduling it when we enter teleop and cancelling
    // it when we exit.  We'll use the same logic for autonomous, once
    // we get there.
    var driveCommand: Command? = null
    override fun teleopInit() {
        driveCommand = TeleopDriveCommand(
            Controls::getDroneControllerDriveInputs
        )
        driveCommand!!.schedule()
    }
    override fun teleopExit() {
        driveCommand?.cancel()
    }

    override fun teleopPeriodic() {
        // code to run every timestep *in teleop*
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll() // this was in the template
    }
}
