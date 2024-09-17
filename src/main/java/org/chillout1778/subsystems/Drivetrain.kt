package org.chillout1778.subsystems

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.ReplanningConfig
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Velocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants
import org.chillout1778.Robot
import org.chillout1778.commands.DriveCommand
import kotlin.math.PI

object Drivetrain : Subsystem, Sendable{

    val kIdleMode : IdleMode = IdleMode.kBrake
    val stallLimit: Int = 40
    val freeLimit: Int = 40
    val rightMaster = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = kIdleMode
        setSmartCurrentLimit(stallLimit, freeLimit)
    }
    val rightSlave = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = kIdleMode
        setSmartCurrentLimit(stallLimit, freeLimit)
    }
    val leftMaster = CANSparkMax(Constants.Drivetrain.LEFT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = kIdleMode
        setSmartCurrentLimit(stallLimit, freeLimit)
    }
    val leftSlave = CANSparkMax(Constants.Drivetrain.LEFT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = kIdleMode
        setSmartCurrentLimit(stallLimit, freeLimit)
    }

    init{
        rightMaster.inverted = false
        leftMaster.inverted = true
        rightSlave.follow(rightMaster)
        leftSlave.follow(leftMaster)
    }

    private val gyro = AHRS()
    private val drivetrain = DifferentialDrive(leftMaster, rightMaster)
    private val odometry = DifferentialDriveOdometry(
        gyro.rotation2d,
        leftMaster.toDrivetrainDistance(),
        rightMaster.toDrivetrainDistance()
    )
    private val kinematics = DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH)

    val factor = Math.PI / 30.0 * Constants.Drivetrain.WHEEL_RADIUS * Constants.Drivetrain.REDUCTION

    private fun CANSparkMax.toDrivetrainDistance() : Measure<Distance>{
        return Meters.of(this.encoder.position * Constants.Drivetrain.REDUCTION * 2.0 * PI * Constants.Drivetrain.WHEEL_RADIUS)
    }
    private fun CANSparkMax.toDrivetrainSpeed(): Measure<Velocity<Distance>>{
        return MetersPerSecond.of(this.encoder.velocity / 60.0 * Constants.Drivetrain.REDUCTION * 2.0 * PI * Constants.Drivetrain.WHEEL_RADIUS)
    }
    init{
        rightMaster.getEncoder().setVelocityConversionFactor(factor)
        leftMaster.getEncoder().setVelocityConversionFactor(factor)
    }

    fun drive(driveSpeed: Double, rotationSpeed: Double){
        drivetrain.arcadeDrive(driveSpeed, rotationSpeed)
        odometry.update(
            gyro.rotation2d,
            DifferentialDriveWheelPositions(
                leftMaster.toDrivetrainDistance(),
                rightMaster.toDrivetrainDistance()
            )
        )
    }

    fun zeroYaw(){
        gyro.zeroYaw()
    }

    init {
        defaultCommand = DriveCommand()
    }

    private fun resetPosition(pose: Pose2d){
        odometry.resetPosition(
            gyro.rotation2d,
            leftMaster.toDrivetrainDistance(),
            rightMaster.toDrivetrainDistance(),
            pose
        )
    }

    fun configureAutoBuilder(){
        AutoBuilder.configureRamsete(
            { odometry.poseMeters },
            { pose : Pose2d -> resetPosition(pose) },
            { kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
                leftMaster.toDrivetrainSpeed(),
                rightMaster.toDrivetrainSpeed())) },
            { speeds: ChassisSpeeds ->
                drive(speeds.vxMetersPerSecond / Constants.Drivetrain.MAX_SPEED,
                    speeds.omegaRadiansPerSecond / Constants.Drivetrain.MAX_ANGULAR_SPEED) },
             ReplanningConfig(),
            { Robot.redAlliance },
            this
        )
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("Gyro", {gyro.rotation2d.degrees}, {})
    }

    init{
        Shuffleboard.getTab("Drivetrain").add("Drivetrain", this)
    }
}