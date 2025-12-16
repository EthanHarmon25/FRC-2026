package frc.robot.Subsystems;

// -------------------- Imports --------------------
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.SwerveConstants;


//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// -------------------------------------------------
public class SwerveModule {

    // --------------------
    // Motors and Encoders
    // --------------------
    public SparkMax driveMotor;
    public SparkMax steerMotor;
    public RelativeEncoder driveEncoder;
    public RelativeEncoder steerEncoder;
    public CANcoder absoluteEncoder;

    // --------------------
    // Config flags
    // --------------------
    private boolean driveInverted;
    private boolean steerInverted;
    private double angleOffsetRadians;

    // --------------------
    // Imported Control
    // --------------------

    public PIDController pivotController;
    public PIDController velocityDriveController;

    // --------------------
    // Constructor
    // --------------------
    public SwerveModule(
        int driveID, 
        int steerID, 
        int encoderID,
        boolean driveInverted, 
        boolean steerInverted, 
        double angleOffsetRadians
    ) {
        this.driveInverted = driveInverted;
        this.steerInverted = steerInverted;
        this.angleOffsetRadians = angleOffsetRadians;

        // --------------------
        // Drive Motor Setup
        // --------------------
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .inverted(driveInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.DRIVE_POS_CONVERSION)
            .velocityConversionFactor(SwerveConstants.DRIVE_VEL_CONVERSION);
        driveConfig.closedLoop
            .pid(1.0, 0.0, 0.0); // default drive PID

        
        driveEncoder = driveMotor.getEncoder();

        // --------------------
        // Steer Motor Setup
        // --------------------
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
            .inverted(steerInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        steerConfig.encoder
            .positionConversionFactor(SwerveConstants.STEER_POS_CONVERSION);
        steerConfig.closedLoop
            .pid(1.0, 0.0, 0.0); // default steer PID

        
        steerEncoder = steerMotor.getEncoder();

        // --------------------
        // Absolute Encoder Setup
        // --------------------
        absoluteEncoder = new CANcoder(encoderID);
        CANcoderConfiguration absConfig = new CANcoderConfiguration();
        absoluteEncoder.getConfigurator().apply(absConfig);
    }

    // --------------------
    // Encoder Access
    // --------------------
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    public double getAbsoluteAngleRadians() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(getAbsoluteAngleRadians())
        );
    }

    // --------------------
    // Set Wheel State
    // --------------------
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the wheel angle to avoid unnecessary rotation
        SwerveModuleState optimized = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromRadians(getAbsoluteAngleRadians())
        );

        driveMotor.set(optimized.speedMetersPerSecond / SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        steerMotor.set(optimized.angle.getRadians());
    }

    // --------------------
    // Stop the Module
    // --------------------
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    // --------------
    // Helper Methods
    // -------------
    public boolean getIsDriveInverted() {
        return this.driveInverted;
    }

    public boolean getIsSteerInverted() {
        return this.steerInverted;
    }

    public double getAngleOffsetRadians() {
        return this.angleOffsetRadians;
    }
}
