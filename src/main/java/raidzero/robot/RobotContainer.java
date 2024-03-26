package raidzero.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import raidzero.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    public static final XboxController driver = new XboxController(0);

    // Driver joystick axes
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    // Autochooser
    private final SendableChooser<Command> chooser;

    // Subsystem references
    private final raidzero.robot.subsystems.Swerve swerve = raidzero.robot.subsystems.Swerve.system();

    public RobotContainer() {
        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", chooser);

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}