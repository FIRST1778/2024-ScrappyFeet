package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants

object Elevator : Subsystem {
    //master on right, positive up
    private val masterMotor = TalonFX(Constants.Elevator.MASTER_MOTOR_ID)
    private val slaveMotor = TalonFX(Constants.Elevator.SLAVE_MOTOR_ID).apply{ setControl(Follower(Constants.Elevator.MASTER_MOTOR_ID, true)) }

    private val controller = Constants.Elevator.CONTROLLER
    val position get() = masterMotor.position.value * Constants.Elevator.REDUCTION * Constants.Elevator.CONVERSION_FACTOR
    var setpoint: Double = 0.0
        set(position) {
            val temp = position.coerceIn(Constants.Elevator.LOWER_LIMIT, Constants.Elevator.UPPER_LIMIT)
            field = temp
        }

    var zeroed = false

    override fun periodic() {
        if(zeroed) masterMotor.setVoltage(controller.calculate(position, setpoint).coerceIn(-12.0, 12.0))
        else masterMotor.setVoltage(0.0)
    }

    fun resetController(){
        controller.reset(0.0)
    }

    fun zero(){
        masterMotor.setPosition(0.0)
    }
}