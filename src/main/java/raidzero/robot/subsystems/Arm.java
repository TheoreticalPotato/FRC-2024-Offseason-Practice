package raidzero.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import static raidzero.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private TalonFX arm;
    private TalonFXConfiguration armConfig;
    private MotionMagicConfigs motionMagicConfigs;
    private Slot0Configs slot0Configs;

    private TalonFX follow;
    private TalonFXConfiguration followConfig;

    private static Arm armSys = new Arm();

    private Arm() {
        System.out.println("Arm Subsystem init");

        armConfig = new TalonFXConfiguration();

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // TODO: Find these values
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html#using-motion-magic-in-api
        slot0Configs = armConfig.Slot0;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;

        motionMagicConfigs = armConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        // TODO: Soft limits
        // This is ctre version
        // arm.setSoftLimit(SoftLimitDirection.kReverse, -28);
        // arm.enableSoftLimit(SoftLimitDirection.kReverse, true);

        armConfig.HardwareLimitSwitch.ForwardLimitEnable = true;

        arm = new TalonFX(ARM_MOTOR_ID);
        arm.getConfigurator().apply(armConfig);

        followConfig = new TalonFXConfiguration();

        followConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        followConfig.HardwareLimitSwitch.ForwardLimitEnable =

        follow = new TalonFX(ARM_FOLLOW_ID);
        follow.getConfigurator().apply(followConfig);
        follow.setControl(new Follower(ARM_MOTOR_ID, true));
    }

    public void trapezoidToPID(State output) {
        pid.setReference(output.position, CANSparkMax.ControlType.kPosition);// 0,
                                                                             // FEED_FORWARD.calculate(output.position,
                                                                             // output.velocity));
        SmartDashboard.putNumber("Arm Trapazoid setpoint", output.position);
    }

    public State currentState() {
        return new State(arm.getEncoder().getPosition(), arm.getEncoder().getVelocity());
    }

    public void stopMotors() {
        arm.stopMotor();
    }

    public void setPos(double setpoint) {
        pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("processVariable", encoder.getPosition());
    }

    public void home() {
        arm.set(0.4);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public boolean getLimit() {
        if (limit1.isPressed() || limit2.isPressed())
            encoder.setPosition(0);
        return limit1.isPressed() || limit2.isPressed();
    }

    @Override
    public void periodic() {
    }

    public static Arm system() {
        return armSys;
    }
}