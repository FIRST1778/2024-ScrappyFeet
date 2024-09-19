package org.chillout1778

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.Pair
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.chillout1778.commands.*
import org.chillout1778.subsystems.Drivetrain
import org.chillout1778.subsystems.Elevator
/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {
    private var autoChooser: SendableChooser<Command> = SendableChooser()
    private var autonomousCommand: Command? = null
    val redAlliance : Boolean get() = DriverStation.getAlliance().get() == DriverStation.Alliance.Red

    enum class Autos(val command: Command) {
        SHOOT_GET_SPIKE_SHOOT(
            SequentialCommandGroup(
                DriveForwardCommand(0.0, 0.0, 5.0)
            )
        )
    }

    override fun robotInit() {
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        Drivetrain
        Controls
        CommandScheduler.getInstance().registerSubsystem(Elevator)
//        Drivetrain.zeroYaw()
//        Drivetrain.configureAutoBuilder()
//        configureNamedCommands()
        autoChooser.setDefaultOption("Nothing", InstantCommand())
        for(auto in Autos.entries) autoChooser.addOption(auto.name, auto.command)
        Shuffleboard.getTab("Autos").add("Auto", autoChooser)

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {
        autonomousCommand = autoChooser.selected
        ElevatorZeroCommand().andThen(ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)).andThen(autonomousCommand).schedule()
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
        ElevatorZeroCommand().andThen(ElevatorMoveCommand(Constants.Elevator.ElevatorState.STORED)).schedule()
        LimelightFlashCommand().schedule()
    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {

    }

    fun configureNamedCommands(){
        NamedCommands.registerCommands(
            listOf(
                Pair("Shoot", ShooterShootCommand()),
                Pair("Intake", IntakeCommand())
            )
        )
    }


}