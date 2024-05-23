package raidzero.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;

public class Wrist {
    private TalonFX leadFx;
    private TalonFX followFx;

    private TalonFXConfiguration config;
    private HardwareLimitSwitchConfigs limitConfigs;

    private static Wrist wristSys = new Wrist();

    private Wrist(){
        leadFx = new TalonFX(Constants.Arm.WRIST_LEAD_ID, "rio");
        followFx = new TalonFX(Constants.Arm.WRIST_FOLLOW_ID);

        limitConfigs = new HardwareLimitSwitchConfigs().
        withForwardLimitSource(ForwardLimitSourceValue.RemoteCANifier);

        leadFx.setInverted(true);

        followFx.setControl(new Follower(leadFx.getDeviceID(), true));

        config = new TalonFXConfiguration().withHardwareLimitSwitch(limitConfigs);

        leadFx.getConfigurator().apply(config);
    }

    public void valueMove(double speed){
        leadFx.set(speed);
    }

    public void getLimit(){
        SmartDashboard.putNumber("wristLimit", leadFx.getReverseLimit().getValue().value);
    }

    public static Wrist getSystem(){
        return wristSys;
    }
}
