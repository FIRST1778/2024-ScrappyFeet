package org.chillout1778

object Utils {
    fun deadZone(inpt : Double, zone : Double): Double {
        return if (Math.abs(inpt) < zone) 0.0 else ((inpt - zone) / (1.0 - zone))
    }
}