package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.Swerve

class TeleopDriveCommand(val driver: CommandXboxController): Command() {
    val deadband: Double = 0.1

    override fun execute() {
        // With the WPILib coordinate system, x is forward and y is to
        // the left.  (This is for the blue side, so TODO later we'll
        // worry about inverting this for red.)
        val x = driver.leftY     // forward
        val y = -driver.leftX    // left
        val rotation = driver.rightX  // counterclockwise

        // Convert (x,y) into polar coordinates so that we can
        // manipulate magnitude (r) instead of separately manipulating
        // x and y.  This works better for squaring and for deadbands.
        val theta = Math.atan(y/x)
        var r = Math.hypot(x, y)

        // Clamp or deadband r if necessary.  The value r will always be
        // positive because theta represents the angle and r the
        // unsigned distance from the origin.
        r = if (r > 1.0) {
            1.0 // clamp to 1
        } else if (r < 0.1) {
            0.0 // round to 0
        } else {
            // Otherwise, do a fancy deadband: smoothly interpolate
            // values from 0.1 to 1.0 into the fixed range 0.0 to 1.0.
            // This fixes a problem where going from 0.09 to 0.1 causes
            // the deadband to suddenly deactivate, thus jumping the
            // output from 0.0 to 0.1.
            // https://web.archive.org/web/20181021234413/http://www.gamasutra.com/blogs/JoshSutphin/20130416/190541/Doing_Thumbstick_Dead_Zones_Right.php
            (r - deadband) / (1.0 - deadband)
        }

        // Square the distance.  All values are between 0 and 1, so
        // squaring will make them smaller.  (Except that 1*1 = 1 so
        // that won't change.)  This approach gives the driver better
        // control when going slowly.
        //
        // Our code avoids the following scenario: if the maximum
        // joystick input lies on a circle (e.g., Xbox controller),
        // *and* you square X and Y separately, then you won't
        // be able to go as fast diagonally as you can go straight
        // forward.  See the following issue on YAGSL:
        // https://github.com/BroncBotz3481/YAGSL-Example/issues/196
        r = r*r

        // Now convert back to rectangular coordinates.
        val newX = r * Math.cos(theta)
        val newY = r * Math.sin(theta)

        Swerve.driveFieldRelative(x = newX, y = newY, rotation = rotation)
    }
}
