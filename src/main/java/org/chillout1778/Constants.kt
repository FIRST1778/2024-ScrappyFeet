package org.chillout1778

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object Constants {
    object Drivetrain{
        const val RIGHT_NEO_MASTER_ID = 3
        const val RIGHT_NEO_SLAVE_ID = 4
        const val LEFT_NEO_MASTER_ID = 1
        const val LEFT_NEO_SLAVE_ID = 2
        const val PIGEON_ID = 30

        val TRACK_WIDTH = Units.inchesToMeters(22.9)
        val WHEEL_RADIUS = Units.inchesToMeters(3.0)
        const val REDUCTION = 1.0 / 7.31

        val MAX_SPEED = 5676.0 / 60.0 * REDUCTION * 2.0 * PI * WHEEL_RADIUS // m/s
        val MAX_ANGULAR_SPEED: Double = MAX_SPEED / (TRACK_WIDTH / 2) // rad/s
    }
    object Shooter{
        const val ROLLER_MOTOR_ID = 5
        const val TOP_FLY_WHEEL_MOTOR_ID = 6
        const val BOTTOM_FLY_WHEEL_MOTOR_ID = 7
    }
    object CenteringWheels{
        const val RIGHT_MOTOR_ID = 8
        const val LEFT_MOTOR_ID = 9
    }
    object Elevator{
        const val MASTER_MOTOR_ID = 10
        const val SLAVE_MOTOR_ID = 11
        val CONTROLLER = ProfiledPIDController(100.0, 0.0, 0.0, TrapezoidProfile.Constraints(40.0, 60.0))
        const val REDUCTION = 1/12.444
        const val CONVERSION_FACTOR = .12 //meters/pulley revolution
        const val LOWER_LIMIT = 0.0
        val UPPER_LIMIT = Units.inchesToMeters(31.0) //can technically be 31.25

        const val POSITION_TOLERANCE = .01

        enum class ElevatorState(var position: Double){
            STORED(0.01),
            CLIMB_UP(Units.inchesToMeters(31.0)),
            CLIMB_DOWN(Units.inchesToMeters(15.0)),
            AMP(Units.inchesToMeters(31.0))
        }
    }
}