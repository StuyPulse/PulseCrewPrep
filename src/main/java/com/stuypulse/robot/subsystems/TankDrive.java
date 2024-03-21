/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.stuypulse.robot.constants.Motors;
//import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TankDrive extends SubsystemBase {
    
    // array of right and left motors 
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    // might need to comment this if no navx
    private final AHRS navx;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;
    private final DifferentialDrive tankDrive;

    //encoder ports
    private final int LEFT_A = 0;
    private final int LEFT_B = 1;
    private final int RIGHT_A = 2;
    private final int RIGHT_B = 3;

    //drivetrain ports
    private final int LEFT_TOP = 0;
    private final int LEFT_MIDDLE = 1;
    private final int LEFT_BOTTOM = 2;

    private final int RIGHT_TOP = 3;
    private final int RIGHT_MIDDLE = 4;
    private final int RIGHT_BOTTOM = 5;
    
    
    public TankDrive() {
        leftMotors = 
            new CANSparkMax[] {
                new CANSparkMax(LEFT_TOP, MotorType.kBrushless),
                new CANSparkMax(LEFT_MIDDLE, MotorType.kBrushless),
                new CANSparkMax(LEFT_BOTTOM, MotorType.kBrushless)
            };
            
        rightMotors = 
            new CANSparkMax[] {
                new CANSparkMax(RIGHT_TOP, MotorType.kBrushless),
                new CANSparkMax(RIGHT_MIDDLE, MotorType.kBrushless),
                new CANSparkMax(RIGHT_BOTTOM, MotorType.kBrushless)
            };

        for (int i = 1; i < leftMotors.length; i++) {
            leftMotors[i].follow(leftMotors[0]);
            rightMotors[i].follow(rightMotors[0]);
        }

        leftEncoder = new Encoder(LEFT_A, LEFT_B);
        rightEncoder = new Encoder(RIGHT_A, RIGHT_B);

        tankDrive =
            new DifferentialDrive(
                leftMotors[0],
                rightMotors[0]
            );
        navx = new AHRS(SPI.Port.kMXP);

        odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field = new Field2d();  
        reset(getPose());
        
        setMotorConfig(Motors.TankDrive.LEFT, Motors.TankDrive.RIGHT);
    }

    private boolean ENCODER_INVERTED = true;

    private void setMotorConfig(Motors.Config left, Motors.Config right) {
        leftEncoder.setReverseDirection(
            ENCODER_INVERTED ^ left.INVERTED); 
        for(CANSparkMax motor : leftMotors) {
            left.configure(motor);
        }

        rightEncoder.setReverseDirection(
            ENCODER_INVERTED ^ right.INVERTED);
        for(CANSparkMax motor : rightMotors) {
            right.configure(motor);
        }
    }
 
    // private void setEncoderDistancePerPulse(double distance) {
    //     rightEncoder.setDistancePerPulse(distance);
    //     rightEncoder.reset();

    //     leftEncoder.setDistancePerPulse(distance);
    //     leftEncoder.reset();
    // }

   /*********************
     * ENCODER FUNCTIONS *
     *********************/

    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /***************
     * ROBOT ANGLE *
     ***************/

    //Try to code Angle.fromDegrees without Stuylibs

    //Gets current Angle of the Robot as a double (contiuous / not +-180)
    public Rotation2d getRawGyroAngle() {
        return navx.getRotation2d();
    }

    // Gets current Angle of the Robot
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(getRawEncoderAngle());
    }

    // Gets current Angle of the Robot as a double [using encoders] (contiuous / not +-180)
    private double getRawEncoderAngle() {
        double distance = getLeftDistance() - getRightDistance();
        return Math.toDegrees(distance / Settings.TankDrive.TRACK_WIDTH);
    }

    // Gets current Angle of the Robot [using encoders]
    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(getRawEncoderAngle());
    }

    public Rotation2d getAngle() {
        return Settings.TankDrive.USING_GYRO ? getGyroAngle() : getEncoderAngle();
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(navx.getRoll());
    }

    public void stop() {
        tankDrive.stopMotor();
    }

    public void tankDrive(double left, double right) {
        tankDrive.tankDrive(left, right, false);
    }

    public void arcadeDrive(double speed, double rotation) {
        tankDrive.arcadeDrive(speed, rotation, false);
    }

    public void curvatureDrive(double xSpeed, double zRotation) {
        tankDrive.curvatureDrive(xSpeed, zRotation, ENCODER_INVERTED);
    }


    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    private void updateOdometry() {
        odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    public Field2d getField() {
        return field;
    }

    public void reset(Pose2d location) {
        navx.reset();
        leftEncoder.reset();
        rightEncoder.reset();

        //might need
        //odometry.resetPosition(location, DifferentialDriveWheelPositions, getRotation2d());
    }

    public void reset() {
        reset(getPose());
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        // Smart Dashboard Information
        if (Settings.DEBUG_MODE) {

            SmartDashboard.putNumber("Debug/TankDrive/Roll (deg)", getRoll().getDegrees());

            SmartDashboard.putData("Debug/TankDrive/Field", field);
            SmartDashboard.putNumber("Debug/TankDrive/Odometer X Position (m)", getPose().getX());
            SmartDashboard.putNumber("Debug/TankDrive/Odometer Y Position (m)", getPose().getY());
            SmartDashboard.putNumber("Debug/TankDrive/Odometer Rotation (deg)", getPose().getRotation().getDegrees());

            // SmartDashboard.putNumber("Debug/TankDrive/Motor Voltage Left (V)", getLeftVoltage());
            // SmartDashboard.putNumber("Debug/TankDrive/Motor Voltage Right (V)", getRightVoltage());

            SmartDashboard.putNumber("Debug/TankDrive/Distance Traveled (m)", getDistance());
            SmartDashboard.putNumber(
                    "Debug/TankDrive/Distance Traveled Left (m)", getLeftDistance());
            SmartDashboard.putNumber(
                    "Debug/TankDrive/Distance Traveled Right (m)", getRightDistance());

            SmartDashboard.putNumber("Debug/TankDrive/Velocity (m per s)", getVelocity());
            SmartDashboard.putNumber("Debug/TankDrive/Velocity Left (m per s)", getLeftVelocity());
            SmartDashboard.putNumber(
                    "Debug/TankDrive/Velocity Right (m per s)", getRightVelocity());

            // SmartDashboard.putNumber("Debug/TankDrive/Current Left (amps)", getLeftCurrentAmps());
            // SmartDashboard.putNumber("Debug/TankDrive/Current Right (amps)", getRightCurrentAmps());

            SmartDashboard.putNumber("Debug/TankDrive/Angle NavX (deg)", getAngle().getDegrees());
            SmartDashboard.putNumber("Debug/TankDrive/Encoder Angle (deg)", getEncoderAngle().getDegrees());
        }
    }
}
