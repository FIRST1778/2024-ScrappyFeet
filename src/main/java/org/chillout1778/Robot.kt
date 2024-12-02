package org.chillout1778

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.subsystems.Swerve

object Robot : TimedRobot() {
    override fun robotInit() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)

        // Bind Triggers (e.g., the operator controller's A button) to
        // Commands (e.g., IntakeCommand).
        Controls.bindTriggers()
        // Note: Last year we did this in Controls' init{} block, and
        // here we had a line that just read "Controls", which would
        // run the initializers for the Controls object, thus binding
        // the triggers.  But I think it's confusing that accessing
        // Controls would run all that code.  This year, I want to put
        // it in its own function (bindTriggers) that we always call in
        // the correct place (here).
        Swerve
    }

    override fun robotPeriodic() {
        // This line activates WPILib's Command framework, which takes
        // care of all the Subsystems and Commands.
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        // For now, we reset gyro angle ourselves and assume we're
        // pointing forwards.  Later, PathPlanner will reset it for us.
        Swerve.robotAngle = 0.0
        Swerve.odometry.resetPosition(Rotation2d(), Swerve.getAllModulePositions(), Pose2d())
    }

    // These two functions make sure the TeleopDriveCommand is running
    // during teleop, and that it stops running once we leave teleop.
    override fun teleopInit() {
        Swerve.defaultCommand = TeleopDriveCommand(Controls::getDroneControllerDriveInputs)
    }
    override fun teleopExit() {
        Swerve.removeDefaultCommand()
    }

    // This function was in the WPILib template, and it's the only one
    // I'm scared to remove.
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
