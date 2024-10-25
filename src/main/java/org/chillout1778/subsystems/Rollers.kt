package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Subsystem

object Rollers: Subsystem {
    val motor = TalonFX(4)

    enum class State(val speed: Double) {
        SUCK(0.4),
        SPIT(-0.4),
        STOP(0.0)
    }

    fun setState(state: State) {
        motor.set(state.speed)
    }
}