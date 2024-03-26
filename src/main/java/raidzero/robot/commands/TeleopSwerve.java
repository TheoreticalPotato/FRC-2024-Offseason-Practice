package raidzero.robot.commands;

import raidzero.robot.Constants.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private raidzero.robot.subsystems.Swerve swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private double translationVal;
    private double strafeVal;
    private double rotationVal;

    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup,DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = raidzero.robot.subsystems.Swerve.system();
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Swerve.STICK_DEADBAND);
        strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Swerve.STICK_DEADBAND);

        swerve.drive(
                new Translation2d(translationVal * translationVal * translationVal, strafeVal * strafeVal * strafeVal).times(Swerve.MAX_SPEED_MPS),
                rotationVal * rotationVal * rotationVal * Swerve.MAX_ANGULAR_VELOCITY,
                true,
                true);
    }
}