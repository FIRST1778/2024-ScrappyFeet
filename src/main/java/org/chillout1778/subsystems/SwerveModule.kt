package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import org.chillout1778.Constants

class SwerveModule(
    driveMotorID: Int,
    turnMotorID: Int,
    canCoderID: Int,
    val driveInverted: InvertedValue,
    val turnInverted: InvertedValue
) {
    val driveMotor: TalonFX
    val turnMotor: TalonFX
    val canCoder: CANcoder

    init {
        driveMotor = TalonFX(driveMotorID)
        turnMotor = TalonFX(turnMotorID)
        canCoder = CANcoder(canCoderID)

        driveMotor.configurator.apply(
            TalonFXConfiguration()
        )
        turnMotor.configurator.apply(
            TalonFXConfiguration()
        )
        driveMotor.configurator.apply(
            MotorOutputConfigs().withInverted(driveInverted),
        )
        turnMotor.configurator.apply(
            MotorOutputConfigs().withInverted(turnInverted)
        )
    }

    val turnPID = Constants.Swerve.makeTurnPID()

    fun turnPosition(): Double {
        return turnMotor.position.valueAsDouble * 2 * Math.PI
    }

    fun drive(angle: Double, driveVelocity: Double) {
        val turnVoltage = turnPID.calculate(turnPosition(), angle)
        turnMotor.setVoltage(turnVoltage)
        driveMotor.setVoltage(driveVelocity * Constants.Swerve.MAX_SPEED / 12)
    }
}
