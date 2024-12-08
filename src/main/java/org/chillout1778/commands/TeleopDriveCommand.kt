package org.chillout1778.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Supplier
import org.chillout1778.Controls
import org.chillout1778.Controls.DriveInputs
import org.chillout1778.subsystems.Swerve
import org.chillout1778.Utils
import kotlin.math.*

class TeleopDriveCommand(
    private val driveInputsSupplier: Supplier<DriveInputs>
) : Command() {
    init {
        addRequirements(Swerve)
    }

    override fun execute() {
        // With the WPILib coordinate system, x is forward and y is to
        // the left.  This is for the blue side, so TODO: invert this
        // for red alliance.
        val inputs = driveInputsSupplier.get()
        val x = inputs.forward
        val y = inputs.left
        var rotation = inputs.rotation

        // Convert (x,y) into polar coordinates so that we can
        // manipulate magnitude (r) instead of separately manipulating
        // x and y.  This works better for squaring and for deadbands.
        val theta = atan2(y, x)
        var r = hypot(x, y)

        // Clamp or deadband r if necessary.  The value r will always be
        // positive because theta represents the angle and r the
        // unsigned distance from the origin.
        r = Utils.deadZone(r, 0.1)
        rotation = Utils.deadZone(rotation, 0.1)

        // Square the distance.  All values are between 0 and 1, so
        // squaring will make them smaller.  (Except that 1*1 = 1 so
        // that won't change.)  This approach gives the driver better
        // control when going slowly.
        //
        // Note that you shouldn't square X and Y separately:
        // https://github.com/BroncBotz3481/YAGSL-Example/issues/196
        r = r*r
        rotation = rotation*rotation*Math.signum(rotation)

        // Now convert back to rectangular coordinates.
        val actualX = r * cos(theta) * Swerve.Constants.MAX_VELOCITY
        val actualY = r * sin(theta) * Swerve.Constants.MAX_VELOCITY
        val actualRotation = rotation * Swerve.Constants.MAX_ANGULAR_VELOCITY

        Swerve.driveFieldRelative(
            ChassisSpeeds(actualX*5, actualY*5, actualRotation*5)
        )
    }
}
