package org.chillout1778

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import kotlin.math.abs

fun neo(id: Int): CANSparkMax {
    val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
    motor.restoreFactoryDefaults()
    // motor.burnFlash()
    return motor
}

fun deadband(n: Double) = if(abs(n) < 0.05) 0.0 else n
