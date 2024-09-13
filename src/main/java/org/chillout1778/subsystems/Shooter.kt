package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants

object Shooter : Subsystem {
    private val rollerMotor = TalonFX(Constants.Shooter.ROLLER_MOTOR_ID)
    private val topFlywheelMotor = TalonFX(Constants.Shooter.TOP_FLY_WHEEL_MOTOR_ID)
    private val bottomFlywheelMotor = TalonFX(Constants.Shooter.BOTTOM_FLY_WHEEL_MOTOR_ID).apply{
        setControl(Follower(Constants.Shooter.TOP_FLY_WHEEL_MOTOR_ID, false))
    }
    private val lineBreak = DigitalInput(0)
    val noteStored get() = !lineBreak.get()
    val atFlywheelSpeed get() = topFlywheelMotor.velocity.value > 4000.0

    fun suck(){
        rollerMotor.set(-0.3)
    }
    fun spit(){
        rollerMotor.set(0.5)
    }
    fun slowSpit(){
        rollerMotor.set(0.1)
    }
    fun stopRollers(){
        rollerMotor.set(0.0)
    }
    fun revFlywheels(){
        topFlywheelMotor.setVoltage(10.0)
    }
    fun stopFlywheels(){
        topFlywheelMotor.set(0.0)
    }
}