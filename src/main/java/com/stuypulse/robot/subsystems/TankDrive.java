/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

// package com.stuypulse.robot.subsystems;

// import com.stuypulse.robot.constants.Motors;
// import com.stuypulse.robot.constants.Ports;
// import com.stuypulse.robot.constants.Settings;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.kinematics.Odometry;
// import edu.wpi.first.units.Angle;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.kauailabs.navx.frc.AHRS;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;


// /*-
//  * Moves the robot around
//  *
//  * Contains:
//  *      - 3 motors on left side
//  *      - 3 motors on right side
//  *      - Encoders for both sides
//  *      - NavX / Gyroscope
//  */
// public class TankDrive extends SubsystemBase {

//     // An array of motors on the left and right side of the drive train
//     private final CANSparkMax[] leftMotors;
//     private final CANSparkMax[] rightMotors;

//     // DifferentialDrive
//     private final DifferentialDrive TankDrive;

//     // An encoder for each side of the drive train
//     private final Encoder leftGrayhill;
//     private final Encoder rightGrayhill;

//     // NAVX for Gyro
//     private final AHRS navx;

//     // Odometry
//     private final DifferentialDriveOdometry odometry;
//     private final Field2d field;

//     public TankDrive() {
//         // Add Motors to list
//         leftMotors =
//                 new CANSparkMax[] {
//                     new CANSparkMax(Ports.TankDrive.LEFT_TOP, MotorType.kBrushless),
//                     new CANSparkMax(Ports.TankDrive.LEFT_MIDDLE, MotorType.kBrushless),
//                     new CANSparkMax(Ports.TankDrive.LEFT_BOTTOM, MotorType.kBrushless)
//                 };

//         rightMotors =
//                 new CANSparkMax[] {
//                     new CANSparkMax(Ports.TankDrive.RIGHT_TOP, MotorType.kBrushless),
//                     new CANSparkMax(Ports.TankDrive.RIGHT_MIDDLE, MotorType.kBrushless),
//                     new CANSparkMax(Ports.TankDrive.RIGHT_BOTTOM, MotorType.kBrushless)
//                 };

//         for (int i = 1; i < leftMotors.length; i++) {
//             leftMotors[i].follow(leftMotors[0]);
//             rightMotors[i].follow(rightMotors[0]);
//         }
//         // Make differential drive object
//         TankDrive =
//                 new DifferentialDrive(
//                     leftMotors[0],
//                     rightMotors[0]
//                 );                        

//     // Create Encoders
//     leftGrayhill = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B);
//     rightGrayhill = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);
//     setGrayhillDistancePerPulse(Encoders.GRAYHILL_DISTANCE_PER_PULSE);

//         // Initialize NAVX
//         navx = new AHRS(SPI.Port.kMXP);

//         // Initialize Odometry
//         odometry = new DifferentialDriveOdometry(getRotation2d());
//         field = new Field2d();
//         reset(Odometry.STARTING_POSITION);

//         // Configure Motors and Other Things
//         setMotorConfig(Motors.TankDrive.LEFT, Motors.TankDrive.RIGHT);
//     }

//     /***********************
//      * MOTOR CONFIGURATION *
//      ***********************/

// private void setMotorConfig(Motors.Config left, Motors.Config right) {
//     leftGrayhill.setReverseDirection(
//             Settings.TankDrive.Encoders.GRAYHILL_INVERTED ^ left.INVERTED);
//     for (CANSparkMax motor : leftMotors) {
//         left.configure(motor);
//     }

//         rightGrayhill.setReverseDirection(
//                 Settings.TankDrive.Encoders.GRAYHILL_INVERTED ^ right.INVERTED);
//         for (CANSparkMax motor : rightMotors) {
//             right.configure(motor);
//         }
//     }

//     private void setGrayhillDistancePerPulse(double distance) {
//         rightGrayhill.setDistancePerPulse(distance);
//         rightGrayhill.reset();

//         leftGrayhill.setDistancePerPulse(distance);
//         leftGrayhill.reset();
//     }

//     /*********************
//      * ENCODER FUNCTIONS *
//      *********************/

//     // Distance
//     public double getLeftDistance() {
//         return leftGrayhill.getDistance();
//     }

//     public double getRightDistance() {
//         return rightGrayhill.getDistance();
//     }

//     public double getDistance() {
//         return (getLeftDistance() + getRightDistance()) / 2.0;
//     }

//     // Velocity
//     public double getLeftVelocity() {
//         return leftGrayhill.getRate();
//     }

//     public double getRightVelocity() {
//         return rightGrayhill.getRate();
//     }

//     public double getVelocity() {
//         return (getLeftVelocity() + getRightVelocity()) / 2.0;
//     }

//     /***************
//      * ROBOT ANGLE *
//      ***************/

//     // Gets current Angle of the Robot as a double (contiuous / not +-180)
//     public double getRawGyroAngle() {
//         return navx.getAngle();
//     }

//     // Gets current Angle of the Robot
//     public Angle getGyroAngle() {
//         return Angle.fromDegrees(getRawGyroAngle());
//     }

//     // Gets current Angle of the Robot as a double [using encoders] (contiuous / not +-180)
//     private double getRawEncoderAngle() {
//         double distance = getLeftDistance() - getRightDistance();
//         return Math.toDegrees(distance / Settings.TankDrive.TRACK_WIDTH);
//     }

//     // Gets current Angle of the Robot [using encoders]
//     public Angle getEncoderAngle() {
//         return Angle.fromDegrees(getRawEncoderAngle());
//     }

//     public Angle getAngle() {
//         return Settings.TankDrive.USING_GYRO ? getGyroAngle() : getEncoderAngle();
//     }

//     public Angle getRoll() {
//         return Angle.fromDegrees(navx.getRoll());
//     }

//     /**********************
//      * ODOMETRY FUNCTIONS *
//      **********************/

//     private void updateOdometry() {
//         odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
//     }

//     public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//         return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
//     }

//     public Rotation2d getRotation2d() {
//         // TODO: check if this needs to be negative
//         return getAngle().negative().getRotation2d();
//     }

//     public Pose2d getPose() {
//         updateOdometry();
//         return odometry.getPoseMeters();
//     }

//     public Field2d getField() {
//         return field;
//     }

//     /************************
//      * OVERALL SENSOR RESET *
//      ************************/

//     public void reset(Pose2d location) {
//         navx.reset();
//         leftGrayhill.reset();
//         rightGrayhill.reset();

//         //might need
//         //odometry.resetPosition(location, DifferentialDriveWheelPositions, getRotation2d());
//     }

//     public void reset() {
//         reset(getPose());
//     }

//     /*********************
//      * VOLTAGE FUNCTIONS *
//      *********************/

//     public double getBatteryVoltage() {
//         return RobotController.getBatteryVoltage();
//     }

//     public double getLeftVoltage() {
//         return leftMotors[0].get() * getBatteryVoltage();
//     }

//     public double getRightVoltage() {
//         return rightMotors[0].get() * getBatteryVoltage();
//     }

//     public void tankDriveVolts(double leftVolts, double rightVolts) {
//         for (MotorController motor : leftMotors) {
//             motor.setVoltage(leftVolts);
//         }

//         for (MotorController motor : rightMotors) {
//             motor.setVoltage(rightVolts);
//         }

//         TankDrive.feed();
//     }

//     /********************
//      * DRIVING COMMANDS *
//      ********************/

//     // Stops TankDrive from moving
//      public void stop() {
//          TankDrive.stopMotor();
//      }

//     // Drives using tank drive
//     public void tankDrive(double left, double right) {
//         TankDrive.tankDrive(left, right, false);
//      }

//     // Drives using arcade drive
//     public void arcadeDrive(double speed, double rotation) {
//         TankDrive.arcadeDrive(speed, rotation, false);
//     }

//     // Drives using curvature drive algorithm
//     public void curvatureDrive(double xSpeed, double zRotation, double baseTS) {
//         // Clamp all inputs to valid values
//         xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
//         zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
//         baseTS = MathUtil.clamp(baseTS, 0.0, 1.0);

//         // Find the amount to slow down turning by.
//         // This is proportional to the speed but has a base value
//         // that it starts from (allows turning in place)
//         double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

//         // Find the speeds of the left and right wheels
//         double lSpeed = xSpeed + zRotation * turnAdj;
//         double rSpeed = xSpeed - zRotation * turnAdj;

//         // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
//         // it will be scaled down proportionally with the other wheels.
//         double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

//         lSpeed /= scale;
//         rSpeed /= scale;

//         // Feed the inputs to the TankDrive
//         tankDrive(lSpeed, rSpeed);
//     }

//     // Drives using curvature drive algorithm with automatic quick turn
//     public void curvatureDrive(double xSpeed, double zRotation) {
//         this.curvatureDrive(xSpeed, zRotation, Settings.TankDrive.BASE_TURNING_SPEED.get());
//     }

//     /*********************
//      * DEBUG INFORMATION *
//      *********************/

//     @Override
//     public void periodic() {
//         updateOdometry();
//         field.setRobotPose(getPose());

//         // Smart Dashboard Information
//         if (Settings.DEBUG_MODE.get()) {

//             SmartDashboard.putNumber("Debug/TankDrive/Roll (deg)", getRoll().toDegrees());

//             SmartDashboard.putData("Debug/TankDrive/Field", field);
//             SmartDashboard.putNumber("Debug/TankDrive/Odometer X Position (m)", getPose().getX());
//             SmartDashboard.putNumber("Debug/TankDrive/Odometer Y Position (m)", getPose().getY());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Odometer Rotation (deg)",
//                     getPose().getRotation().getDegrees());

//             SmartDashboard.putNumber("Debug/TankDrive/Motor Voltage Left (V)", getLeftVoltage());
//             SmartDashboard.putNumber("Debug/TankDrive/Motor Voltage Right (V)", getRightVoltage());

//             SmartDashboard.putNumber("Debug/TankDrive/Distance Traveled (m)", getDistance());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Distance Traveled Left (m)", getLeftDistance());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Distance Traveled Right (m)", getRightDistance());

//             SmartDashboard.putNumber("Debug/TankDrive/Velocity (m per s)", getVelocity());
//             SmartDashboard.putNumber("Debug/TankDrive/Velocity Left (m per s)", getLeftVelocity());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Velocity Right (m per s)", getRightVelocity());

//             SmartDashboard.putNumber("Debug/TankDrive/Current Left (amps)", getLeftCurrentAmps());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Current Right (amps)", getRightCurrentAmps());

//             SmartDashboard.putNumber("Debug/TankDrive/Angle NavX (deg)", getAngle().toDegrees());
//             SmartDashboard.putNumber(
//                     "Debug/TankDrive/Encoder Angle (deg)", getEncoderAngle().toDegrees());
//         }
//     }
// }


//curvature thing 
// public void curvatureDrive(double xSpeed, double zRotation, double baseTS) {
    //     // Clamp all inputs to valid values
    //     xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    //     zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    //     baseTS = MathUtil.clamp(baseTS, 0.0, 1.0);

    //     // Find the amount to slow down turning by.
    //     // This is proportional to the speed but has a base value
    //     // that it starts from (allows turning in place)
    //     double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

    //     // Find the speeds of the left and right wheels
    //     double lSpeed = xSpeed + zRotation * turnAdj;
    //     double rSpeed = xSpeed - zRotation * turnAdj;

    //     // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
    //     // it will be scaled down proportionally with the other wheels.
    //     double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

    //     lSpeed /= scale; 
    //     rSpeed /= scale; 

    //     // Feed the inputs to the TankDrive
    //     tankDrive(lSpeed, rSpeed);
    // }