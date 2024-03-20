package com.stuypulse.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    public int leftMotorPort = 0;
    public int rightMotorPort = 1;

    private double targetLeftRPM;
    private double targetRightRPM; 
 
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SimpleMotorFeedforward leftController;
    private SimpleMotorFeedforward rightController;

    public Shooter() {
        leftMotor = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

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
        return leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getTargetLeftRPM() {
        return targetLeftRPM;
    }

    public double getTargetRightRPM() {
        return targetRightRPM;
    }

    public double getLeftVoltage() {
        return leftMotor.getBusVoltage();
    }

    public double getRightVoltage() {
        return rightMotor.getBusVoltage();
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
