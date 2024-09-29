package org.chillout1778.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.neo

object CenteringWheels: Subsystem {
    const val RIGHT_ID = 8
    const val LEFT_ID = 9

    private val rightMotor = neo(RIGHT_ID)
    private val leftMotor = neo(LEFT_ID).apply {
        follow(rightMotor, true)
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
