package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

class SwerveModule(
    driveMotorID: Int,
    turnMotorID: Int,
    canCoderID: Int,
    canCoderOffsetDegrees: Double,
    driveInverted: InvertedValue,
    turnInverted: InvertedValue,
) : Sendable {
    object Constants {
        val WHEEL_RADIUS = Units.inchesToMeters(2.0)
        fun makeTurnPID() = PIDController(0.4, 0.0, 0.01)
        fun makeDrivePID() = PIDController(0.0, 0.0, 0.0)
        fun makeDriveFeedforward() = SimpleMotorFeedforward(0.0, 1.0, 0.0)
    }

    private val driveMotor: TalonFX = TalonFX(driveMotorID)
    private val turnMotor: TalonFX = TalonFX(turnMotorID)
    private val canCoder: CANcoder = CANcoder(canCoderID)
    init {
        driveMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(1.0 / 5.35714285714)
                MotorOutput = MotorOutputConfigs().withInverted(driveInverted)
            }
        )
        turnMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(7.0 / 150.0)
                MotorOutput = MotorOutputConfigs().withInverted(turnInverted)
            }
        )
        turnMotor.setPosition(canCoder.absolutePosition.valueAsDouble + canCoderOffsetDegrees/360.0)
    }

    private val turnPID = Constants.makeTurnPID()
    private val drivePID = Constants.makeDrivePID()
    private val driveFeedforward = Constants.makeDriveFeedforward()

    private val turnPosition: Double
        get() = turnMotor.position.valueAsDouble * 2*PI
    private val driveVelocity: Double
        get() = driveMotor.velocity.valueAsDouble * 2*PI * Constants.WHEEL_RADIUS
    private val driveAcceleration: Double
        get() = driveMotor.acceleration.valueAsDouble * 2*PI * Constants.WHEEL_RADIUS

    fun driveState(state: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromRadians(turnPosition))
        val goalTurnPosition = optimizedState.angle.radians
        val goalDriveVelocity = optimizedState.speedMetersPerSecond * cos(turnPosition - goalTurnPosition)
        turnMotor.setVoltage(turnPID.calculate(turnPosition, goalTurnPosition))
        driveMotor.setVoltage(drivePID.calculate(driveVelocity, goalDriveVelocity)
            + driveFeedforward.calculate(goalDriveVelocity, driveAcceleration))
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.addDoubleProperty("turn position (deg)", {Math.toDegrees(turnPosition)}, {})
        builder.addDoubleProperty("drive velocity (m/s)", {driveVelocity}, {})
        builder.addDoubleProperty("drive acceleration (m/s/s)", {driveAcceleration}, {})
    }
}
