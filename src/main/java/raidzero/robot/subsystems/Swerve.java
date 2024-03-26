package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static raidzero.robot.Constants.Swerve.*;

import raidzero.robot.Constants;
import raidzero.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    
    private Pigeon2 imu;
    public SwerveModule moduleFL, moduleBL, moduleBR, moduleFR;

    public SwerveDriveOdometry swerveOdometry;

    private static Swerve swerveSys = new Swerve();

    private Swerve() {
        System.out.println("Swerve Subsystem Init");

        moduleFL = new SwerveModule(
            THROTTLE_FL_ID,
            ROTOR_FL_ID,
            CAN_CODER_FL_ID,
            MODULE_FL_OFFSET,
            false
        );

        moduleBL = new SwerveModule(
            THROTTLE_BL_ID,
            ROTOR_BL_ID,
            CAN_CODER_BL_ID,
            MODULE_BL_OFFSET,
            false
        );

        moduleBR = new SwerveModule(
            THROTTLE_BR_ID,
            ROTOR_BR_ID,
            CAN_CODER_BR_ID,
            MODULE_BR_OFFSET,
            true
        );

        moduleFR = new SwerveModule(
            THROTTLE_FR_ID,
            ROTOR_FR_ID,
            CAN_CODER_FR_ID,
            MODULE_FR_OFFSET,
            true
        );


        imu = new Pigeon2(PIGEON_ID, Constants.Swerve.CANBUS_ID);
        imu.getConfigurator().apply(new Pigeon2Configuration());
        imu.setYaw(0);

        swerveOdometry = new SwerveDriveOdometry(
            SWERVE_DRIVE_KINEMATICS,
            getRotation(),
            getModulePositions()
        );

        configureAutoBuilder();
    }

    /**
     * Drives the swerve
     * 
     * @param translation XY velocities
     * @param rotation Rotation velocity
     * @param fieldRelative Field relative driving
     * @param isOpenLoop Open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                                        getHeading())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED_MPS);

        moduleFL.setDesiredState(swerveModuleStates[0], isOpenLoop);
        moduleFR.setDesiredState(swerveModuleStates[1], isOpenLoop);
        moduleBL.setDesiredState(swerveModuleStates[2], isOpenLoop);
        moduleBR.setDesiredState(swerveModuleStates[3], isOpenLoop);
    }

    /**
     * Sets swervemodule states
     * 
     * @param desiredStates Desired swerve module states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED_MPS);

        // SmartDashboard.putNumber("input vel mps", desiredStates[0].speedMetersPerSecond);

        moduleFL.setDesiredState(desiredStates[0], false);
        moduleFR.setDesiredState(desiredStates[1], false);
        moduleBL.setDesiredState(desiredStates[2], false);
        moduleBR.setDesiredState(desiredStates[3], false);

    }

    public Command setX() {
        return runOnce( () -> {
            moduleFL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
		    moduleBL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
		    moduleBR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
		    moduleFR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
        });
	}

    /**
     * Gets swervemodule states
     * 
     * @return Module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = moduleFL.getState();
        states[1] = moduleFR.getState();
        states[2] = moduleBL.getState();
        states[3] = moduleBR.getState();

        return states;
    }

    /**
     * Gets swervemodule positions
     * 
     * @return Module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = moduleFL.getPosition();
        positions[1] = moduleFR.getPosition();
        positions[2] = moduleBL.getPosition();
        positions[3] = moduleBR.getPosition();

        return positions;
    }

    /**
     * Gets robot pose
     * 
     * @return Robot pose
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Sets robot pose
     * 
     * @param pose Desired pose
     */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Gets robot heading as a {@link Rotation2d} object
     * 
     * @return Robot heading as {@link Rotation2d} object
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Sets robot heading
     * 
     * @param heading {@link Rotation2d} heading
     */
    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Zeros robot heading
     */
    public void zeroHeading() {
        swerveOdometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets IMU rotation as a {@link Rotation2d} object
     * 
     * @return IMU rotation as {@link Rotation2d} object
     */
    public Rotation2d getRotation() {
        return imu.getRotation2d();
    }

    /**
     * Reset swervemodules to absolute position
     */
    public void resetModulesToAbsolute() {
        moduleFL.resetToAbsolute();
        moduleBL.resetToAbsolute();
        moduleBR.resetToAbsolute();
        moduleFR.resetToAbsolute();
    }

    public double getHeadingOrdinalTurn(){
        return Math.IEEEremainder(imu.getRotation2d().getDegrees(), 360);
    }

    /**
     * Gets robot relative speeds
     * 
     * @return Robot speeds as {@link ChassisSpeeds} object
     */
    public ChassisSpeeds getRelativeSpeeds() {
        return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Pigeon2 getImu() {
        return imu;
    }

    /**
     * <ul>
     * <li>Robot relative driving</li>
     * <li>Used for PathPlanner holonomic driving</li>
     * </ul>
     * 
     * @param speed Speed as {@link ChassisSpeeds} object
     */
    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speed));
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRelativeSpeeds,
            this::driveRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    Constants.Swerve.TRANSLATION_KP,
                    Constants.Swerve.TRANSLATION_KI,
                    Constants.Swerve.TRANSLATION_KD
                ),
                new PIDConstants(
                    Constants.Swerve.ROTATION_KP,
                    Constants.Swerve.ROTATION_KI,
                    Constants.Swerve.ROTATION_KD
                ),
                Constants.Swerve.MAX_SPEED_MPS,
                Constants.Swerve.TRACK_WIDTH / 2.0,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            this
        );
    }

    public static Swerve system() {
        return swerveSys;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getRotation(), getModulePositions());
    }
}