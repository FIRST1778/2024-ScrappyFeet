package org.chillout1778

import kotlin.math.abs

object Utils {
    fun deadZone(inpt: Double, zone: Double): Double {
        return if (abs(inpt) < zone) 0.0
            else if (inpt > 1.0) 1.0
            else if (inpt < -1.0) -1.0
            else ((inpt - zone) / (1.0 - zone))
        // Fancy deadzone: interpolate values from 0.1 to 1.0 into the
        // range 0.0 to 1.0.
        // https://web.archive.org/web/20181021234413/http://www.gamasutra.com/blogs/JoshSutphin/20130416/190541/Doing_Thumbstick_Dead_Zones_Right.php
    }
}
