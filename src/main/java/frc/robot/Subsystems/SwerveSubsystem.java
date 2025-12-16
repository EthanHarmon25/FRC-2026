// 
package frc.robot.Subsystems;

// -------------------- Imports --------------------
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

//1647 Use the Swerve constants.java
import frc.robot.Constants.SwerveConstants;

// -------------------------------------------------
/**
 * Controls the entire swerve drivetrain:
 *  - integrates 4 swerve modules
 *  - reads gyro for field orientation
 *  - maintains odometry
 *  - converts drive/strafe/rotation commands into module states
 */

public class SwerveSubsystem extends SubsystemBase {

    // -------------------- Module Instances --------------------
    //1647 Setup Swerve Modules
    private final SwerveModule FL;
    private final SwerveModule FR;
    private final SwerveModule BL;
    private final SwerveModule BR;
    //1647 -> Setup Gyro
    private final Pigeon2 gyro;

    // 1647 -> Setup Kinematics
    private final SwerveDriveKinematics kinematics;
    // 1647 -> Setup Odometry
    private final SwerveDriveOdometry odometry;

    // -------------------- Constructor --------------------
    public SwerveSubsystem() {
        //1647 Impliment Gyro
        gyro = new Pigeon2(SwerveConstants.PIGEON_ID, SwerveConstants.CAN_BUS);
        
        //1647 Impliment Front Left
        FL = new SwerveModule(
            SwerveConstants.FL_DRIVE_ID,
            SwerveConstants.FL_STEER_ID,
            SwerveConstants.FL_ENCODER_ID,
            SwerveConstants.DRIVE_INVERTED,
            SwerveConstants.STEER_INVERTED,
            0.0 // offset placeholder, set after calibration
        );

        //1647 Impliment Front Right
        FR = new SwerveModule(
            SwerveConstants.FR_DRIVE_ID,
            SwerveConstants.FR_STEER_ID,
            SwerveConstants.FR_ENCODER_ID,
            SwerveConstants.DRIVE_INVERTED,
            SwerveConstants.STEER_INVERTED,
            0.0
        );

        //1647 Impliment Back Left
        BL = new SwerveModule(
            SwerveConstants.FL_DRIVE_ID,
            SwerveConstants.FL_STEER_ID,
            SwerveConstants.FL_ENCODER_ID,
            SwerveConstants.DRIVE_INVERTED,
            SwerveConstants.STEER_INVERTED,
            0.0
        );

        //1647 Impliment Back Right
        BR = new SwerveModule(
            SwerveConstants.BR_DRIVE_ID,
            SwerveConstants.BR_STEER_ID,
            SwerveConstants.BR_ENCODER_ID,
            SwerveConstants.DRIVE_INVERTED,
            SwerveConstants.STEER_INVERTED,
            0.0
        );

        //1647 Impliment Swerve Kinamatics
        //(Setup Distance between Swerve Modules on Chassis)
        kinematics = new SwerveDriveKinematics(
            SwerveConstants.FL_LOCATION,
            SwerveConstants.FR_LOCATION,
            SwerveConstants.BL_LOCATION,
            SwerveConstants.BR_LOCATION
        );

        //1647 Impliment Odometry
        //()
        odometry = new SwerveDriveOdometry(
            kinematics,
            //1647 Get rotational pos in deg from gyro
            getHeading(),
            //1647 Get rotational position of each swerve module as yaw
            getModulePositions()
        );
    }

    // -------------------- Helper Accessors --------------------
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        };
    }

    // -------------------- Odometry Update --------------------
    @Override
    public void periodic() {
        odometry.update(getHeading(), getModulePositions());
    }

    // -------------------- Drive Control --------------------
    /**
     * Drives the robot given desired chassis speeds.
     *
     * @param fwd  Forward velocity (m/s)
     * @param str  Strafe (sideways) velocity (m/s)
     * @param rot  Angular velocity (rad/s)
     * @param fieldOriented Whether to drive relative to field or robot
     */
    public void drive(double fwd, double str, double rot, boolean fieldOriented) {
        //1647 If chassisSpeed is field oriented, then do ?, else do : and create a new one
        // 1647 note: set up object for inverse kinematics below 
        ChassisSpeeds chassisSpeeds = fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, getHeading())
            : new ChassisSpeeds(fwd, str, rot);

        //1647 what the hell are these two lines - ethan
        // 1647 this line performs inverse kinematics (aka "swerve math")
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        // 1647 this line performs normalization to limit to the physical limits (like max speed)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);

        FL.setDesiredState(states[0]);
        FR.setDesiredState(states[1]);
        BL.setDesiredState(states[2]);
        BR.setDesiredState(states[3]);
    }

    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }
}

