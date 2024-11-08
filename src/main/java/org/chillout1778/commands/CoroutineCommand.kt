package org.chillout1778.commands
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.coroutines.*
import kotlin.coroutines.cancellation.CancellationException

abstract class CoroutineCommand(vararg requirements: Subsystem): Command() {
    init {
        addRequirements(*requirements)
    }

    // Implement this function in your derived class.
    abstract suspend fun runRoutine()

    suspend fun wait(predicate: () -> Boolean) {
        while (!predicate())
            yield()
    }

    suspend fun waitForever() {
        while (true) yield()
    }

    suspend fun waitSeconds(seconds: Double) {
        val timer = Timer()
        timer.start()
        wait { timer.hasElapsed(seconds)}
    }

    suspend fun waitTicks(n: Int) {
        repeat(n) {
            yield()
        }
    }

    private var continuation: Continuation<Unit>? = null

    suspend fun yield() = suspendCoroutine { cont ->
        continuation = cont
    }

    override fun initialize() {
        continuation = this::runRoutine.createCoroutine(
            Continuation(
                context = EmptyCoroutineContext,
                resumeWith = { result ->
                    result.onFailure { exception ->
                        if (!(exception is CancellationException)) {
                            println("caught exception thrown in coroutine: $exception")
                        }
                    }
                    continuation = null
                }
            )
        )
        continuation!!.resume(Unit)
    }

    override fun execute() {
        continuation!!.resume(Unit)
    }

    override fun isFinished() = continuation == null

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            continuation!!.resumeWithException(
                CancellationException("command canceled by WPILib", cause = null)
            )
        }
    }
}