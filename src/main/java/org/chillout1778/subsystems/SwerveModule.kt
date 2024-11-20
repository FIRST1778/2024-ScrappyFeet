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
        // Colsons have a diameter of 4 inches.
        val WHEEL_RADIUS = Units.inchesToMeters(2.0)
        // These turn PID values are borrowed straight from Zappy.
        fun makeTurnPID() = PIDController(0.4, 0.0, 0.01)
        // Theoretically the code can do closed-loop control, but to
        // start with, I've set the PID constants to 0 so that they
        // contribute no voltage.
        fun makeDrivePID() = PIDController(0.0, 0.0, 0.0)
        // This feedforward is clever in that it does the same thing as
        // open-loop control.  kS and kA are zero, so we don't add any
        // voltage to overcome static friction or acceleration; it's
        // all from the kV term (calculated as kV * velocity). Our
        // current kV value is 12 / max_velocity, so the total voltage
        // is velocity / max_velocity * 12, which is just like
        // open-loop control.
        fun makeDriveFeedforward() = SimpleMotorFeedforward(
            0.0,
            12.0 / Swerve.Constants.MAX_VELOCITY,
            0.0
        )
        // I think these are right.
        val DRIVE_RATIO = 1.0 / 5.35714285714
        val TURN_RATIO = 7.0 / 150.0
    }

    private val driveMotor: TalonFX = TalonFX(driveMotorID)
    private val turnMotor: TalonFX = TalonFX(turnMotorID)
    private val canCoder: CANcoder = CANcoder(canCoderID)
    init {
        driveMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(Constants.DRIVE_RATIO)
                MotorOutput = MotorOutputConfigs().withInverted(driveInverted)
            }
        )
        turnMotor.configurator.apply(
            TalonFXConfiguration().apply {
                Feedback = FeedbackConfigs().withSensorToMechanismRatio(Constants.TURN_RATIO)
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
        builder.addDoubleProperty("raw drive position (rotations)", {driveMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("drive velocity (m/s)", {driveVelocity}, {})
        builder.addDoubleProperty("drive acceleration (m/s/s)", {driveAcceleration}, {})
    }
}
