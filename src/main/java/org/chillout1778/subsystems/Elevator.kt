package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.abs

object Elevator : Subsystem, Sendable {
    val REDUCTION = 1/12.444
    val CONVERSION_FACTOR = 0.12 //meters/pulley revolution
    val LOWER_LIMIT = 0.0
    val UPPER_LIMIT = Units.inchesToMeters(31.0) //can technically be 31.25
    val POSITION_TOLERANCE = .01
    val MASTER_ID = 11
    val SLAVE_ID = 10

    //master on right, positive up
    private val masterMotor = TalonFX(MASTER_ID)

    init {
        TalonFX(SLAVE_ID).apply { setControl(Follower(MASTER_ID, true)) } // true = inverted
    }

    private val controller = ProfiledPIDController(100.0, 0.0, 0.0, TrapezoidProfile.Constraints(40.0, 60.0))

    private val position get() = masterMotor.position.value * REDUCTION * CONVERSION_FACTOR
    private val rawPosition get() = masterMotor.position.value

    var climbed = false // once we go to half mast, we should never put the elevator down

    private var setpoint: Double = 0.0
    fun up()   { setpoint = UPPER_LIMIT }
    fun half() { setpoint = Units.inchesToMeters(15.0); climbed = true }
    fun down() { if (!climbed) setpoint = LOWER_LIMIT }

    val atSetpoint get() = abs(position - setpoint) <= POSITION_TOLERANCE
    
    private var zeroed = false
    fun ensureZeroed() {
        if (!zeroed) {
            masterMotor.setPosition(0.0)
            controller.reset(0.0)
            zeroed = true
        }
    }

    override fun periodic() {
        if (zeroed) {
            masterMotor.setVoltage(controller.calculate(position, setpoint).coerceIn(-12.0, 12.0))
        } else {
            masterMotor.setVoltage(0.0)
        }
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("Position", {Units.metersToInches(position)}, {})
        builder.addDoubleProperty("input voltage", { masterMotor.motorVoltage.value}, {})
        builder.addBooleanProperty("Zeroed", {zeroed}, {})
//        builder.addStringProperty("Command", {currentCommand.name}, {})
    }

    init{
        Shuffleboard.getTab("Elevator").add("Elevator", this)
    }
}
