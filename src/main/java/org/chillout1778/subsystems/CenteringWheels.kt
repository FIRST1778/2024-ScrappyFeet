package org.chillout1778.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants

object CenteringWheels : Subsystem {
    private val rightMotor = CANSparkMax(Constants.CenteringWheels.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val leftMotor = CANSparkMax(Constants.CenteringWheels.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless).apply {
        follow(rightMotor,true)
    }
    fun suck(){
        rightMotor.set(0.5)
    }
    fun spit(){
        rightMotor.set(-0.8)
    }
    fun passiveSpit(){
        rightMotor.set(-.1)
    }
    fun stop(){
        rightMotor.set(0.0)
    }
}
