package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.ReplanningConfig
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants
import org.chillout1778.Robot
import org.chillout1778.commands.DriveCommand
import java.util.function.Supplier
import kotlin.math.PI

object Drivetrain : Subsystem {
    val rightMaster = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless)
    val rightSlave = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless)
    val leftMaster = CANSparkMax(Constants.Drivetrain.LEFT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless)
    val leftSlave = CANSparkMax(Constants.Drivetrain.LEFT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless)

    init{
        rightMaster.inverted = false
        leftMaster.inverted = true
        rightSlave.follow(rightMaster)
        leftSlave.follow(leftMaster)
    }

    private val gyro = Pigeon2(Constants.Drivetrain.PIGEON_ID)
    private val drivetrain = DifferentialDrive(leftMaster, rightMaster)
    private val odometry = DifferentialDriveOdometry(
        gyro.rotation2d,
        rightMaster.encoder.position,
        leftMaster.encoder.position,
    )
    private val kinematics = DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH)

    init{
        val factor = Math.PI / 30.0 * Constants.Drivetrain.WHEEL_RADIUS * Constants.Drivetrain.REDUCTION
        rightMaster.getEncoder().setVelocityConversionFactor(factor)
        leftMaster.getEncoder().setVelocityConversionFactor(factor)
    }

    fun drive(driveSpeed: Double, rotationSpeed: Double){
        drivetrain.arcadeDrive(driveSpeed, rotationSpeed)
    }

    fun zeroYaw(rotation: Double = 0.0){
        gyro.setYaw(Math.toDegrees(rotation))
    }

    init {
        defaultCommand = DriveCommand()
    }

    private fun resetPosition(pose: Pose2d){
        odometry.resetPosition(
            gyro.rotation2d,
            rightMaster.encoder.position,
            leftMaster.encoder.position,
            pose
        )
    }

    fun configureAutoBuilder(){
        AutoBuilder.configureRamsete(
            { odometry.poseMeters },
            { pose : Pose2d -> resetPosition(pose) },
            { kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
                leftMaster.encoder.velocity / 60.0 * Constants.Drivetrain.REDUCTION * 2.0 * PI * Constants.Drivetrain.WHEEL_RADIUS,
                rightMaster.encoder.velocity / 60.0 * Constants.Drivetrain.REDUCTION * 2.0 * PI * Constants.Drivetrain.WHEEL_RADIUS)) },
            { speeds: ChassisSpeeds -> drive(speeds.vxMetersPerSecond / Constants.Drivetrain.MAX_SPEED, speeds.omegaRadiansPerSecond / Constants.Drivetrain.MAX_ANGULAR_SPEED) },
             ReplanningConfig(),
            { Robot.redAlliance },
            this
        )
    }
}