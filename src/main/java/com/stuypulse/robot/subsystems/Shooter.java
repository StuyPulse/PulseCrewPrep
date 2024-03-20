package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private CANSparkMax motor1;
    private CANSparkMax motor2;

    private RelativeEncoder encoder1;
    private RelativeEncoder encoder2;

    private SimpleMotorFeedforward controller1;
    private SimpleMotorFeedforward controller2;

    private double shooterSpeed = 100;

    public Shooter() {
        motor1 = new CANSparkMax(Settings.Shooter.motor1, MotorType.kBrushless);
        motor2 = new CANSparkMax(Settings.Shooter.motor2, MotorType.kBrushless);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        controller1 = new SimpleMotorFeedforward(0, 0);
        controller2 = new SimpleMotorFeedforward(0, 0);
    }

    public void shoot() {
        motor1.set(shooterSpeed);
        motor2.set(-shooterSpeed);
    }

    public void shootStop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    public double getVelocityOne() {
        return encoder1.getVelocity();
    }

    public double getVelocityTwo() {
        return encoder2.getVelocity();
    }

    @Override
    public void periodic() {
        motor1.setVoltage(shooterSpeed);
    }

}
