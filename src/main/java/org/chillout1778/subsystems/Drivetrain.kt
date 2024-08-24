package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants

object Drivetrain : Subsystem {
    val rightMaster = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless)
    val rightSlave = CANSparkMax(Constants.Drivetrain.RIGHT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless)
    val leftMaster = CANSparkMax(Constants.Drivetrain.LEFT_NEO_MASTER_ID,CANSparkLowLevel.MotorType.kBrushless)
    val leftSlave = CANSparkMax(Constants.Drivetrain.LEFT_NEO_SLAVE_ID,CANSparkLowLevel.MotorType.kBrushless)

    init{
        rightSlave.follow(rightMaster)
        leftSlave.follow(leftMaster)
    }

    private val controller = RamseteController(2.0,0.7)
    private val gyro = Pigeon2(Constants.Drivetrain.PIGEON)
    private val kinematics = DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH)
    private val odometry = DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH)

    init{
        val factor = Math.PI / 30.0 * Constants.Drivetrain.WHEEL_RADIUS * Constants.Drivetrain.REDUCTION
        rightMaster.getEncoder().setVelocityConversionFactor(factor)
        leftMaster.getEncoder().setVelocityConversionFactor(factor)
    }

    fun drive(driveSpeed: Double, rotationSpeed: Double){

    }

    fun zeroYaw(rotation: Double = 0.0){
        gyro.setYaw(Math.toDegrees(rotation))
    }
}