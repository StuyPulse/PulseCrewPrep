package com.stuypulse.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTalonFX extends SubsystemBase{
    public int leftMotorPort = 0;
    public int rightMotorPort = 1;

    private double targetLeftRPM;
    private double targetRightRPM; 
 
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private SimpleMotorFeedforward leftController;
    private SimpleMotorFeedforward rightController;

    public ShooterTalonFX() {
        leftMotor = new TalonFX(leftMotorPort);
        rightMotor = new TalonFX(rightMotorPort);

        leftController = new SimpleMotorFeedforward(0, 0);
        rightController = new SimpleMotorFeedforward(0, 0);
    }

    public void shoot(double leftRPM, double rightRPM) {
        targetLeftRPM = leftRPM;
        targetRightRPM = rightRPM;
    }

    public void shootStop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public double getLeftVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    public double getRightVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetLeftRPM() {
        return targetLeftRPM;
    }

    public double getTargetRightRPM() {
        return targetRightRPM;
    }

    public double getLeftVoltage() {
        return leftMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getRightVoltage() {
        return rightMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        leftMotor.setVoltage(leftController.calculate(targetLeftRPM));
        rightMotor.setVoltage(rightController.calculate(targetRightRPM));

        SmartDashboard.putNumber("Shooter/Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Shooter/Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Shooter/Left Target RPM",getTargetLeftRPM());
        SmartDashboard.putNumber("Shooter/Right Target RPM", getTargetRightRPM());
        SmartDashboard.putNumber("Shooter/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Shooter/Right Voltage", getRightVoltage());
    }
}
