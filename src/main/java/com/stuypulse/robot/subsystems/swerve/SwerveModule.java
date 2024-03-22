package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;


public class SwerveModule extends SubsystemBase {
    
    // data
    private final String id;
    private final Translation2d translationOffset;
    private final Rotation2d angleOffset;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor;
    private final CANcoder turnEnoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    // controllers
    private final PIDController driveController;
    private final PIDController turnController;

    public SwerveModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnId, int driveId, int encoderId) {
        this.id = id;
        this.translationOffset = translationOffset;
        this.angleOffset = angleOffset;

        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turnEnoder = new CANcoder(encoderId);
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Swerve.DriveEncoder.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Swerve.DriveEncoder.VELOCITY_CONVERSION);

        driveController = new PIDController(Swerve.Drive.kP, Swerve.Drive.kI, Swerve.Drive.kD);
        turnController = new PIDController(Swerve.Turn.kP, Swerve.Turn.kI, Swerve.Turn.kD);

        targetState = new SwerveModuleState();
    }

    // Constants
    private interface Swerve {
        public interface Drive {
            double kP = 0.018327;
            double kI = 0.0;
            double kD = 0.0;   
        }

        public interface Turn {
            double kP = 6.0;
            double kI = 0.0;
            double kD = 0.15;
        }

        public interface DriveEncoder {
            double WHEEL_DIAMETER = Units.inchesToMeters(4);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
            double GEAR_RATIO = 1.0 / 6.12;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    } 

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEnoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }
    
    public String getID() {
        return id;
    }

    public Translation2d getOffset() {
        return translationOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public void periodic() {
        // need to complete writing this without using StuyLib
    }
}
