package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Flywheels: SubsystemBase() {
    // Krakens: Phoenix 6 (CTRE) - TalonFX
    // NEOs: REVLib (REV) - CANSparkMax
    private val motor = TalonFX(5)

    private val follower = Follower(5, false)
    private val pid = PIDController(0.1, 0.0, 0.0)

    private val gearRatio: Double = 5.0 // "wpilib commands"

    fun velocity(): Double { // Radians per second
        return motor.velocity.valueAsDouble * gearRatio * (2*Math.PI/60)
    }

    fun atSetpoint(): Boolean {
        return Math.abs(velocity() - pid.setpoint) < 0.5
    }

    override fun periodic() {
        val volts = pid.calculate(velocity())
        motor.setVoltage(volts)
    }

    fun slow() {
            pid.setpoint = 0.5
//        motor.set(0.5)  // -1 to 1
    }
    fun fast() {
        pid.setpoint = 2.5
    }
    fun off() {
        pid.setpoint = 0.0
    }
}