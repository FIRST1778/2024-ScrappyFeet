package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.PI
import kotlin.math.cos

class SwerveModule(
    val name: String,
    driveMotorID: Int,
    turnMotorID: Int,
    canCoderID: Int,
    rawCanCoderPositionRotations: Double,
    driveInverted: InvertedValue,
    turnInverted: InvertedValue,
) : Sendable {
    object Constants {
        // Colsons have a diameter of 4 inches.
        val WHEEL_RADIUS = Units.inchesToMeters(2.0)
        // These turn PID values are borrowed straight from Zappy.
        fun makeTurnPID() = PIDController(2.0, 0.0, 0.01).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
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
        val TURN_RATIO =  150.0 / 7.0
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
        canCoder.configurator.apply(
            MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        )
        turnMotor.setPosition(canCoder.absolutePosition.valueAsDouble - rawCanCoderPositionRotations)
        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        turnMotor.setNeutralMode(NeutralModeValue.Brake)
    }

    private val turnPID = Constants.makeTurnPID()
    private val drivePID = Constants.makeDrivePID()
    private val driveFeedforward = Constants.makeDriveFeedforward()

    private val turnPosition: Double
        get() = MathUtil.angleModulus(turnMotor.position.valueAsDouble * 2*PI)
    private val driveVelocity: Double
        get() = driveMotor.velocity.valueAsDouble * 2*PI * Constants.WHEEL_RADIUS
    private val driveAcceleration: Double
        get() = driveMotor.acceleration.valueAsDouble * 2*PI * Constants.WHEEL_RADIUS
    private val drivePosition: Double
        get() = driveMotor.position.valueAsDouble * 2*PI * Constants.WHEEL_RADIUS

    val driveAndTurnPosition get() = SwerveModulePosition(
        drivePosition, Rotation2d(turnPosition)
    )

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

        builder.addDoubleProperty("raw turn position", {turnMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("raw cancoder position", {canCoder.absolutePosition.valueAsDouble}, {})

        builder.addDoubleProperty("turn position (deg)", {Math.toDegrees(turnPosition)}, {})
//        builder.addDoubleProperty("raw drive position (rotations)", {driveMotor.position.valueAsDouble}, {})
        builder.addDoubleProperty("drive velocity (mps)", {driveVelocity}, {})
//        builder.addDoubleProperty("drive acceleration (mps^2)", {driveAcceleration}, {})
    }
}
