package org.chillout1778

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.subsystems.Swerve

object Robot : TimedRobot() {
    fun redAlliance() : Boolean {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    }

    private lateinit var autoChooser: SendableChooser<Command>
    private fun configureAutoChooser(){
        autoChooser = AutoBuilder.buildAutoChooser()
        Shuffleboard.getTab("Autos").apply {
            add(autoChooser).withSize(2, 1)
        }
    }

    lateinit var simonLogger: SimonLogger

    override fun robotInit() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        simonLogger = SimonLogger()
        Controls.bindTriggers()
        Swerve
        configureAutoChooser()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        simonLogger.periodic()
    }

    override fun autonomousInit() {
        Swerve.robotAngle = 0.0
        Swerve.odometry.resetPosition(Rotation2d(), Swerve.getModulePositions(), Pose2d())
        autoChooser.selected.schedule()
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
