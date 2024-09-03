package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants

object Shooter : Subsystem {
    private val rollerMotor = TalonFX(Constants.Shooter.ROLLER_MOTOR_ID)
    private val topFlywheelMotor = CANSparkMax(Constants.Shooter.TOP_FLY_WHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val bottomFlywheelMotor = CANSparkMax(Constants.Shooter.BOTTOM_FLY_WHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless).apply{
        follow(topFlywheelMotor)
    }
    private val lineBreak = DigitalInput(0)
    val noteStored get() = !lineBreak.get()
    val atFlywheelSpeed get() = topFlywheelMotor.encoder.velocity > 4000.0

    fun suck(){
        rollerMotor.set(-0.3)
    }
    fun spit(){
        rollerMotor.set(0.5)
    }
    fun stopRollers(){
        rollerMotor.set(0.0)
    }
    fun revFlywheels(){
        topFlywheelMotor.set(0.8)
    }
    fun stopFlywheels(){
        topFlywheelMotor.set(0.0)
    }
}