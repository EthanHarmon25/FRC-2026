package frc.robot.Constants;

// -------------------- Imports --------------------
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// -------------------------------------------------
/**
 * Contains all tunable and hardware-mapping constants
 * for the 2025 swerve drive system.
 */

public final class SwerveConstants {

    // =============================
    //  Gyro (IMU)
    // =============================
    public static final int PIGEON_ID = 67;          // Change once wired
    public static final String CAN_BUS = "rio";      // CAN bus name for Pigeon2
    
    // =============================
    //  Drivetrain Geometry
    // =============================

    // IMPORTANT 1647 NOTE 
    // Trackwidth is the left to right distnce between left and right swerve modules
    // Wheelbase is the front to back distance between the centers of your front and back swerve modules
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(20.75);
    public static final double WHEELBASE_METERS  = Units.inchesToMeters(20.75);

    // Module positions for kinematics
    public static final Translation2d FL_LOCATION =
        new Translation2d(WHEELBASE_METERS / 2.0,  TRACKWIDTH_METERS / 2.0);
    public static final Translation2d FR_LOCATION =
        new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0);
    public static final Translation2d BL_LOCATION =
        new Translation2d(-WHEELBASE_METERS / 2.0,  TRACKWIDTH_METERS / 2.0);
    public static final Translation2d BR_LOCATION =
        new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0);

    // =============================
    //  Motor & Encoder CAN IDs
    // =============================
    public static final int FL_DRIVE_ID   = 1;
    public static final int FL_STEER_ID   = 2;
    public static final int FL_ENCODER_ID = 3;

    public static final int FR_DRIVE_ID   = 4;
    public static final int FR_STEER_ID   = 5;
    public static final int FR_ENCODER_ID = 6;

    public static final int BL_DRIVE_ID   = 7;
    public static final int BL_STEER_ID   = 8;
    public static final int BL_ENCODER_ID = 9;

    public static final int BR_DRIVE_ID   = 10;
    public static final int BR_STEER_ID   = 11;
    public static final int BR_ENCODER_ID = 12;

    // =============================
    //  Gear Ratios & Wheel Constants
    // =============================
    public static final double DRIVE_GEAR_RATIO = 6.75;   // SDS MK4 L2
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // Conversion factors
    public static final double DRIVE_POS_CONVERSION = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VEL_CONVERSION = DRIVE_POS_CONVERSION / 60.0;
    public static final double STEER_POS_CONVERSION = 2 * Math.PI / STEER_GEAR_RATIO;

    // =============================
    //  Motor Configuration
    // =============================
    public static final boolean DRIVE_INVERTED = false;
    public static final boolean STEER_INVERTED = true;

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int STEER_CURRENT_LIMIT = 30;

    // =============================
    //  PID Tuning Values
    // =============================
    // Drive PID (simple velocity control)
    public static final double DRIVE_P = 0.04;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    // Steer PID (angle control)
    public static final double STEER_P = 0.4;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.01;

    // =============================
    //  Limits & Dynamics
    // =============================
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

    public static final double VOLTAGE_COMPENSATION = 12.0;
    public static final double OPEN_LOOP_RAMP = 0.25;

    private SwerveConstants() {}
}
