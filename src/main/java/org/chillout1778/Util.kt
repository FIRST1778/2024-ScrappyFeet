package org.chillout1778

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

fun neo(id: Int): CANSparkMax {
    val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
    motor.restoreFactoryDefaults()
    return motor
}
