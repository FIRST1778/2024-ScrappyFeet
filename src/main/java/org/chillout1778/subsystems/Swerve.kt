package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.PI
import org.chillout1778.Robot

object Swerve: SubsystemBase() {
    object Constants {
        // How fast the robot can move in a straight line (meters/sec).
        val MAX_VELOCITY = 1//2.62
        // How far the swerve modules are from (0,0).
        val XY_DISTANCE = Units.inchesToMeters(10.365)
        // How fast the robot can rotate (radians/sec).
        val MAX_ANGULAR_VELOCITY = MAX_VELOCITY / XY_DISTANCE / Math.sqrt(2.0)
    }

    val gyro = Pigeon2(30)

    // Read yaw from the gyro (radians, counterclockwise from forward).
    var robotAngle: Double
        get() = MathUtil.angleModulus(-gyro.angle/180.0*Math.PI)
        set(desiredAngle) {
            gyro.setYaw(MathUtil.angleModulus(desiredAngle)/Math.PI*180)
        }

    // Data for all the modules.  They are combined in one big array
    // because that's the "smart" way to do it.  A lot of teams
    // duplicate everything: "frontLeftSwerveModuleDriveMotorId = 1".
    // I hate those variables, the whole point of classes is so we can
    // do better than that.       However...the positions of the modules
    // are listed below in the SwerveKinematics class...so do change that
    // if you change the order.
    //
    // The order shouldn't matter, but PathPlannerLib prefers FL, FR,
    // BL, BR order so we'll just use that.
    private val modules = arrayOf(
        SwerveModule(
            name = "Front Left",
            driveMotorID = 0, turnMotorID = 1, canCoderID = 8,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            rawCanCoderPositionRotations = 0.36279
        ),
        SwerveModule(
            name = "Front Right",
            driveMotorID = 6, turnMotorID = 7, canCoderID = 11,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            rawCanCoderPositionRotations = 0.4797
        ),
        SwerveModule(
            name = "Back Left",
            driveMotorID = 2, turnMotorID = 3, canCoderID = 9,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            rawCanCoderPositionRotations = 0.00264
        ),
        SwerveModule(
            name = "Back Right",
            driveMotorID = 4, turnMotorID = 5, canCoderID = 10,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted  = InvertedValue.Clockwise_Positive,
            rawCanCoderPositionRotations = 0.2492
        )
    )

    init {
        for (module in modules) {
            Shuffleboard.getTab("Swerve").add(module.name, module)
        }
        Shuffleboard.getTab("Swerve").add("swerve actual object", this)
    }

    private val kinematics = SwerveDriveKinematics(
        Translation2d(1.0, 1.0).times(Constants.XY_DISTANCE), // FL
        Translation2d(1.0, -1.0).times(Constants.XY_DISTANCE), // FR
        Translation2d(-1.0, 1.0).times(Constants.XY_DISTANCE), // BL
        Translation2d(-1.0, -1.0).times(Constants.XY_DISTANCE), // BR
    )

    val odometry = SwerveDriveOdometry(
        kinematics, // positions of modules
        Rotation2d(robotAngle), // initial robot yaw (converted to Rotation2d)
        getAllModulePositions() // initial "positions" (how far the wheels have moved
                            // and in what direction)
    )

    fun getAllModulePositions(): Array<SwerveModulePosition> {
        return modules.map { it.driveAndTurnPosition }.toTypedArray()
    }

    fun getAllModuleStates(): Array<SwerveModuleState> {
        return modules.map { SwerveModuleState(it.driveVelocity, Rotation2d(it.turnPosition)) }.toTypedArray()
    }

    fun getOverallSpeed(): Double {
        val speeds = kinematics.toChassisSpeeds(*getAllModuleStates())
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
    }

    fun driveFieldRelative(speeds: ChassisSpeeds) {
        driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Rotation2d.fromRadians(robotAngle)
            )
        )
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.period)
        val moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        for ((mod, state) in modules.zip(moduleStates)) {
            mod.driveState(state)
        }
        odometry.update(Rotation2d(robotAngle), getAllModulePositions())
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("robotAngle", {Math.toDegrees(robotAngle)}, {})
        builder.addDoubleProperty("raw gyro yaw", {gyro.angle}, {})
        builder.addStringProperty("odometry pose", {odometry.poseMeters.toString()}, {})
        builder.addDoubleProperty("overall speed", { getOverallSpeed() }, {})
    }
}
