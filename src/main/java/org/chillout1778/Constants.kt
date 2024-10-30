package org.chillout1778

import edu.wpi.first.math.controller.PIDController

object Constants {
    object Swerve{
        const val FRONT_LEFT_DRIVE_ID = 1
        const val FRONT_LEFT_TURN_ID = 2
        const val FRONT_RIGHT_DRIVE_ID = 3
        const val FRONT_RIGHT_TURN_ID = 4
        const val BACK_RIGHT_DRIVE_ID = 5
        const val BACK_RIGHT_TURN_ID = 6
        const val BACK_LEFT_DRIVE_ID = 7
        const val BACK_LEFT_TURN_ID = 8

        const val FRONT_LEFT_ENCODER_ID = 9
        const val FRONT_RIGHT_ENCODER_ID = 10
        const val BACK_RIGHT_ENCODER_ID = 11
        const val BACK_LEFT_ENCODER_ID = 12

        const val MAX_SPEED = 1

        const val DRIVE_REDUCTION = 1.0 / 5.35714285714
        fun makeTurnPID(): PIDController {
            return PIDController(0.4, 0.0, 0.01)
        }
    }
}