package raidzero.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;

public class Arm {
    private TalonFX leadFx;
    private TalonFX followFx;

    private TalonFXConfiguration config;
    private HardwareLimitSwitchConfigs limitConfigs;

    private static Arm armSys = new Arm();

    private Arm(){
        leadFx = new TalonFX(Constants.Arm.ARM_LEAD_ID);
        followFx = new TalonFX(Constants.Arm.ARM_FOLLOW_ID);

        limitConfigs = new HardwareLimitSwitchConfigs().
        withReverseLimitSource(ReverseLimitSourceValue.RemoteCANifier);

        followFx.setControl(new Follower(leadFx.getDeviceID(), true));

        config = new TalonFXConfiguration().withHardwareLimitSwitch(limitConfigs);

        leadFx.getConfigurator().apply(config);
    }

    public void valueMove(double speed){
        leadFx.set(speed);
    }

    public void getLimit(){
        SmartDashboard.putNumber("armLimit", leadFx.getReverseLimit().getValue().value);
    }

    public static Arm getSystem(){
        return armSys;
    }
}
