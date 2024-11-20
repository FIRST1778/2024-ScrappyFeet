package org.chillout1778


import edu.wpi.first.wpilibj.GenericHID

object Controls {
    val driver = GenericHID(0)

    init {
//        driver.leftTrigger()// Shoot
//            .whileTrue(FlywheelFastCommand())
//        driver.rightTrigger()// Intake
//            .whileTrue(RollerSetCommand(Rollers.State.SUCK))
//        driver.b()// Spit
//            .whileTrue(RollerSetCommand(Rollers.State.SPIT))
    }

}
