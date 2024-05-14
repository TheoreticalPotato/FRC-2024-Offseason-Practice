package raidzero.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Arm {
    private TalonFX arm;
    private TalonFX follower;

    private TalonFXConfiguration config;
    private HardwareLimitSwitchConfigs limitConfigs;

    private static Arm armSys = new Arm();

    private Arm(){
        arm = new TalonFX(0, "seCANdary");
        follower = new TalonFX(0, "seCANdary");

        limitConfigs = new HardwareLimitSwitchConfigs().
        withForwardLimitRemoteSensorID(0);

        follower.setControl(new Follower(arm.getDeviceID(), true));

        config = new TalonFXConfiguration().withHardwareLimitSwitch(limitConfigs);
    }

    private void valueMove(double speed){
        arm.set(speed);
    }
}
