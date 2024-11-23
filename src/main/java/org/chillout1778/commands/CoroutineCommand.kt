package org.chillout1778.commands
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.coroutines.*
import kotlin.coroutines.cancellation.CancellationException

suspend fun contextYield() {
    coroutineContext[CoroutineCommand.CommandCoroutineContext]!!.command.yield()
}

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

    suspend fun waitSeconds(seconds: Double) {
        val timer = Timer()
        timer.start()
        while (!timer.hasElapsed(seconds))
            yield()
    }

    suspend fun waitTicks(n: Int) {
        repeat(n) {
            yield()
        }
    }

    private var continuation: Continuation<Unit>? = null

    data class CommandCoroutineContext(val command: CoroutineCommand)
        : AbstractCoroutineContextElement(Key)
    {
        companion object Key : CoroutineContext.Key<CommandCoroutineContext>
    }

    suspend fun yield() = suspendCoroutine { c ->
        continuation = c
    }

    override fun initialize() {
        continuation = this::runRoutine.createCoroutine(
            Continuation(
                context = CommandCoroutineContext(this),
                resumeWith = { result ->
                    result.onFailure { e ->
                        println("caught exception thrown in coroutine: $e")
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