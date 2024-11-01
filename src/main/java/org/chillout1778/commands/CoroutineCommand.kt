package org.chillout1778.commands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.coroutines.*
import kotlin.coroutines.cancellation.CancellationException
import kotlin.coroutines.intrinsics.*

abstract class CoroutineCommand(vararg requirements: Subsystem): Command() {
    init {
        addRequirements(*requirements)
    }

    // Implement this function in your derived class.
    abstract suspend fun runRoutine()

    suspend fun wait(pred: () -> Boolean) {
        while (!pred())
            yield()
    }

    suspend fun waitForever() {
        while (true) yield()
    }

    suspend fun waitTicks(n: Int) {
        repeat(n) {
            yield()
        }
    }

    private var continuation: Continuation<Unit>? = null

    suspend fun yield() = suspendCoroutineUninterceptedOrReturn { c ->
        continuation = c
        COROUTINE_SUSPENDED
    }

    override fun initialize() {
        continuation = this::runRoutine.createCoroutine(object : Continuation<Unit> {
            override val context: CoroutineContext = EmptyCoroutineContext
            override fun resumeWith(result: Result<Unit>) {
                result.onFailure { e ->
                    println("caught exception thrown in coroutine: $e")
                }
                continuation = null
            }
        })
        continuation!!.resume(Unit)
    }

    override fun execute() {
        continuation!!.resume(Unit)
    }

    override fun isFinished() = continuation == null

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            continuation!!.resumeWithException(CancellationException(
                message = "command was canceled by WPILib",
                cause = null
            ))
        }
    }
}