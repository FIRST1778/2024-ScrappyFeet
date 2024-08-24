package org.chillout1778

import edu.wpi.first.math.util.Units

object Constants {
    object Drivetrain{
        const val RIGHT_NEO_MASTER_ID = 1
        const val RIGHT_NEO_SLAVE_ID = 2
        const val LEFT_NEO_MASTER_ID = 3
        const val LEFT_NEO_SLAVE_ID = 4
        const val PIGEON = 30

        val TRACK_WIDTH = Units.inchesToMeters(22.9)
        val WHEEL_RADIUS = Units.inchesToMeters(6.0)
        const val REDUCTION = 7.31
    }
}