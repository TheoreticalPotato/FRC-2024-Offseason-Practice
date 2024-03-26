package raidzero.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import raidzero.lib.math.Conversions;

public class SwerveModule {

    private TalonFX throttle;
    private TalonFXConfiguration throttleConfig;
    private final SimpleMotorFeedforward throttleFeedForward = new SimpleMotorFeedforward(
            Constants.Swerve.THROTTLE_KS,
            Constants.Swerve.THROTTLE_KV,
            Constants.Swerve.THROTTLE_KA);
    private final DutyCycleOut throttleDutyCycleOut = new DutyCycleOut(0.0);
    private final VelocityVoltage throttleVelocity = new VelocityVoltage(0.0);

    private TalonFX rotor;
    private TalonFXConfiguration rotorConfig;
    private final PositionVoltage rotorPosition = new PositionVoltage(0.0);

    private CANcoder imu;
    public CANcoderConfiguration imuConfig;

    /**
     * Constructs a new SwerveModule
     * 
     * @param throttleID        CAN ID of throttle motor
     * @param rotorID           CAN ID of rotor motor
     * @param canCoderID        CAN ID of CANcoder
     * @param moduleAngleOffset CANcoder offset
     * @param throttleInversion Invert throttle
     */
    public SwerveModule(int throttleID, int rotorID, int canCoderID, double moduleAngleOffset,
            boolean throttleInversion) {

        /* Throttle */
        throttleConfig = new TalonFXConfiguration();

        throttleConfig.MotorOutput.Inverted = Constants.Swerve.THROTTLE_INVERT;
        throttleConfig.MotorOutput.NeutralMode = Constants.Swerve.THROTTLE_NEUTRAL_MODE;

        throttleConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.THROTTLE_GEAR_RATIO;

        throttleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.THROTTLE_CURRENT_LIMIT;
        throttleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        throttleConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD;
        throttleConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.THROTTLE_CURRENT_THRESHOLD_TIME;

        throttleConfig.Slot0.kP = Constants.Swerve.THROTTLE_KP;
        throttleConfig.Slot0.kI = Constants.Swerve.THROTTLE_KI;
        throttleConfig.Slot0.kD = Constants.Swerve.THROTTLE_KD;

        throttleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        throttleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        throttle = new TalonFX(throttleID, Constants.Swerve.CANBUS_ID);
        throttle.getConfigurator().apply(throttleConfig);
        throttle.getConfigurator().setPosition(0.0);

        /* Rotor */
        rotorConfig = new TalonFXConfiguration();

        rotorConfig.MotorOutput.Inverted = Constants.Swerve.ROTOR_INVERT;
        rotorConfig.MotorOutput.NeutralMode = Constants.Swerve.ROTOR_NEUTRAL_MODE;

        rotorConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ROTOR_GEAR_RATIO;
        rotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        rotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ROTOR_ENABLE_CURRENT_LIMIT;
        rotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ROTOR_CURRENT_LIMIT;
        rotorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD;
        rotorConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.ROTOR_CURRENT_THRESHOLD_TIME;

        rotorConfig.Slot0.kP = Constants.Swerve.ROTOR_KP;
        rotorConfig.Slot0.kI = Constants.Swerve.ROTOR_KI;
        rotorConfig.Slot0.kD = Constants.Swerve.ROTOR_KD;

        rotor = new TalonFX(rotorID, Constants.Swerve.CANBUS_ID);
        rotor.getConfigurator().apply(rotorConfig);
        resetToAbsolute();

        /* imu */
        imuConfig = new CANcoderConfiguration();
        imuConfig.MagnetSensor.SensorDirection = Constants.Swerve.CAN_CODER_INVERT;
        imuConfig.withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(moduleAngleOffset));

        imu = new CANcoder(canCoderID, Constants.Swerve.CANBUS_ID);
        imu.getConfigurator().apply(imuConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        rotor.setControl(rotorPosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            throttleDutyCycleOut.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED_MPS;
            throttle.setControl(throttleDutyCycleOut);
        } else {
            throttleVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.WHEEL_CIRCUMFERENCE);
            throttleVelocity.FeedForward = throttleFeedForward.calculate(desiredState.speedMetersPerSecond);
            throttle.setControl(throttleVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(imu.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        rotor.setPosition(getCANcoder().getRotations());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(throttle.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(rotor.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(throttle.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(rotor.getPosition().getValue()));
    }

    public void stopMotors() {
        rotor.stopMotor();
        throttle.stopMotor();
    }
}