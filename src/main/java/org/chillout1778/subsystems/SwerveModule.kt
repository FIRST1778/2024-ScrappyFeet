package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import org.chillout1778.Constants

class SwerveModule(
    driveMotorID: Int,
    turnMotorID: Int,
    canCoderID: Int,
    val inverted: InvertedValue,
    ) {
    val driveMotor : TalonFX
    val turnMotor : TalonFX
    val canCoder : CANcoder
    init {
        driveMotor = TalonFX(driveMotorID)
        turnMotor = TalonFX(turnMotorID)
        canCoder = CANcoder(canCoderID)

        driveMotor.configurator.apply(
            MotorOutputConfigs().withInverted(inverted)
        )
        driveMotor.configurator.apply(
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(1.0)
        )
    }
    val turnPID = Constants.Swerve.turnPID
}