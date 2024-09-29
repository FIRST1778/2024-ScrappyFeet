package org.chillout1778.subsystems

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.ReplanningConfig
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.PI
import org.chillout1778.commands.DriveCommand
import org.chillout1778.neo
import org.chillout1778.Robot

object Drivetrain : Subsystem, Sendable {
    val RIGHT_NEO_MASTER_ID = 3
    val RIGHT_NEO_SLAVE_ID = 4
    val LEFT_NEO_MASTER_ID = 1
    val LEFT_NEO_SLAVE_ID = 2
    val PIGEON_ID = 30

    val TRACK_WIDTH: Double = Units.inchesToMeters(22.9)
    val WHEEL_RADIUS: Double = Units.inchesToMeters(3.0)
    val REDUCTION = 1.0 / 7.31

    val MAX_SPEED = 5676.0 / 60.0 * REDUCTION * 2.0 * PI * WHEEL_RADIUS // m/s
    val MAX_ANGULAR_SPEED: Double = MAX_SPEED / (TRACK_WIDTH / 2) // rad/s

    private fun driveMotor(id: Int) = neo(id).apply {
        idleMode = IdleMode.kBrake
        setSmartCurrentLimit(40, 40) // stall, free
    }
    val rightMaster = driveMotor(RIGHT_NEO_MASTER_ID)
    val rightSlave  = driveMotor(RIGHT_NEO_SLAVE_ID)
    val leftMaster  = driveMotor(LEFT_NEO_MASTER_ID)
    val leftSlave   = driveMotor(LEFT_NEO_SLAVE_ID)

    val factor = Math.PI / 30.0 * WHEEL_RADIUS * REDUCTION

    private fun neoDistance(n: CANSparkMax): Double =
        n.encoder.position * REDUCTION * 2.0 * PI * WHEEL_RADIUS
    private fun neoSpeed(n: CANSparkMax): Double =
        n.encoder.velocity / 60.0 * REDUCTION * 2.0 * PI * WHEEL_RADIUS

    init {
        rightSlave.follow(rightMaster)
        leftSlave.follow(leftMaster)
        rightMaster.inverted = true
        leftMaster.inverted = false
    }

    private val gyro = AHRS()
    private val drivetrain = DifferentialDrive(leftMaster, rightMaster)
    private val odometry = DifferentialDriveOdometry(
        gyro.rotation2d,
        neoDistance(leftMaster), neoDistance(rightMaster))
    private val kinematics = DifferentialDriveKinematics(TRACK_WIDTH)

    init{
        rightMaster.encoder.setVelocityConversionFactor(factor)
        leftMaster.encoder.setVelocityConversionFactor(factor)
    }

    fun drive(driveSpeed: Double, rotationSpeed: Double){
        drivetrain.arcadeDrive(driveSpeed, rotationSpeed)
        odometry.update(
            gyro.rotation2d,
            DifferentialDriveWheelPositions(
                neoDistance(leftMaster),
                neoDistance(rightMaster)
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
            neoDistance(leftMaster),
            neoDistance(rightMaster),
            pose
        )
    }

    fun configureAutoBuilder(){
        AutoBuilder.configureRamsete(
            { odometry.poseMeters },
            { pose : Pose2d -> resetPosition(pose) },
            { kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
                neoSpeed(leftMaster),
                neoSpeed(rightMaster))) },
            { speeds: ChassisSpeeds ->
                drive(speeds.vxMetersPerSecond / MAX_SPEED,
                    speeds.omegaRadiansPerSecond / MAX_ANGULAR_SPEED) },
             ReplanningConfig(),
            { Robot.redAlliance },
            this
        )
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("Gyro", {gyro.rotation2d.degrees}, {})
        builder.addDoubleProperty("Left Speed", { neoSpeed(leftMaster)}, {})
        builder.addDoubleProperty("Right Speed", { neoSpeed(rightMaster)}, {})
        builder.addDoubleProperty("Left Distance", { neoDistance(leftMaster)}, {})
        builder.addDoubleProperty("Right Distance", { neoDistance(rightMaster)}, {})
    }

    init{
        Shuffleboard.getTab("Drivetrain").add("Drivetrain", this)
    }
}
