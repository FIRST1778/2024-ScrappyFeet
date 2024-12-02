package org.chillout1778

import edu.wpi.first.wpilibj.GenericHID

object Controls {
    val droneController = GenericHID(0)

    data class DriveInputs(
        val forward: Double,
        val left: Double,
        val rotation: Double,
    )
    fun getDmytroControllerDriveInputs(): DriveInputs {
        return DriveInputs(
            forward = -droneController.getRawAxis(1),
            left = -droneController.getRawAxis(0),
            rotation = -droneController.getRawAxis(2)
        )
    }

    fun getDroneControllerDriveInputs(): DriveInputs {
        return DriveInputs(
            forward = droneController.getRawAxis(2),
            left = -droneController.getRawAxis(3),
            rotation = -droneController.getRawAxis(0)
        )

    }

    fun bindTriggers() {
        // No Commands to bind yet.
    }

    // private val operatorController = CommandXboxController(1)
    // private fun getOperatorControllerDriveInputs(): DriveInputs {
    //     return DriveInputs(
    //         forward = operatorController.leftY,
    //         left = -operatorController.leftX,
    //         rotation = operatorController.rightX,
    //     )
    // }
}
