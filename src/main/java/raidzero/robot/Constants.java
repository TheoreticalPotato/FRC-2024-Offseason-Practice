package raidzero.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Swerve {

        public static final int PIGEON_ID = 0;

        // Swerve dimensions & conversions
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.0);
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_M * Math.PI;

        public static final double THROTTLE_GEAR_RATIO = (6.12 / 1.0);
        public static final double ROTOR_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        public static final double THROTTLE_VEL_CONVERSION_FACTOR = (1 / THROTTLE_GEAR_RATIO / 60) * WHEEL_DIAMETER_M
                * Math.PI;

        public static final double THROTTLE_POS_CONVERSTION_FACTOR = (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_M
                * Math.PI;

        // Swerve Kinematics
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        // Swerve Current Limiting
        public static final int ROTOR_CURRENT_LIMIT = 25;
        public static final int ROTOR_CURRENT_THRESHOLD = 40;
        public static final double ROTOR_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ROTOR_ENABLE_CURRENT_LIMIT = true;

        public static final int THROTTLE_CURRENT_LIMIT = 40;
        public static final int THROTTLE_CURRENT_THRESHOLD = 60;
        public static final double THROTTLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean THROTTLE_ENABLE_CURRENT_LIMIT = true;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final double OPEN_LOOP_RAMP = 0.25;

        /* Angle Motor PID Values */
        public static final double ROTOR_KP = 100.0;
        public static final double ROTOR_KI = 0.0;
        public static final double ROTOR_KD = 0.0;

        // Throttle PID constants
        public static final double THROTTLE_KP = 0.12;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;// 0.035;
        public static final double THROTTLE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double THROTTLE_KS = 0.32;
        public static final double THROTTLE_KV = 1.51;
        public static final double THROTTLE_KA = 0.27;

        // Translation pathing PID constants
        public static final double TRANSLATION_KP = 0.5; // 0.12
        public static final double TRANSLATION_KI = 0.025;
        public static final double TRANSLATION_KD = 0.0;

        // Rotation pathing PID constants
        public static final double ROTATION_KP = 2.2; // 0.12
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        /* Swerve Profiling Values */
        public static final double THROTTLE_FREE_SPEED = 6000.0;
        public static final double THROTTLE_EFFICIENCY = 0.87;
        public static final double MAX_SPEED_MPS = (THROTTLE_FREE_SPEED * THROTTLE_EFFICIENCY) / THROTTLE_GEAR_RATIO
                * Math.PI * WHEEL_DIAMETER_M / 60.0;
                
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 5.0 * 0.8;

        /* Neutral Modes */
        public static final NeutralModeValue ROTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue THROTTLE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final int THROTTLE_FL_ID = 1;
        public static final int ROTOR_FL_ID = 2;
        public static final int CAN_CODER_FL_ID = 1;
        public static final double MODULE_FL_OFFSET = -0.702881;

        public static final int THROTTLE_BL_ID = 3;
        public static final int ROTOR_BL_ID = 4;
        public static final int CAN_CODER_BL_ID = 2;
        public static final double MODULE_BL_OFFSET = -0.896729;

        public static final int THROTTLE_BR_ID = 5;
        public static final int ROTOR_BR_ID = 6;
        public static final int CAN_CODER_BR_ID = 3;
        public static final double MODULE_BR_OFFSET = -0.525635;

        public static final int THROTTLE_FR_ID = 7;
        public static final int ROTOR_FR_ID = 8;
        public static final int CAN_CODER_FR_ID = 4;
        public static final double MODULE_FR_OFFSET = -0.109375;

        public static final String CANBUS_ID = "seCANdary";

        public static final InvertedValue THROTTLE_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue ROTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CAN_CODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        public static final double STICK_DEADBAND = 0.1;
    }
}