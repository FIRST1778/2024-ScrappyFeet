package org.chillout1778.commands

import edu.wpi.first.wpilibj2.command.SubsystemBase

// Imagine we actually had this object
object Lights: SubsystemBase() {
    fun purple() {}
    fun off() {}
}

class BlinkNumberCommand(var n: Int): CoroutineCommand(Lights) {
    override suspend fun runRoutine() {
        try {
            while (n > 5) {
                n -= 5
                Lights.purple()
                waitTicks(10)
                Lights.off()
                waitTicks(5)
            }
            waitTicks(20)
            while (n > 0) {
                n--
                Lights.purple()
                waitTicks(10)
                Lights.off()
                waitTicks(5)
            }
        } finally {
            Lights.off()
        }
    }
}