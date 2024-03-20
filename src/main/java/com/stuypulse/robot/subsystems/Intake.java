package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final int MOTOR_PORT = 0;
    private final int ACQUIRE_SPEED = 0;

    private final CANSparkMax motor;

    public Intake getInstance(){
        return new Intake();
    }
    
    public Intake() {
        motor = new CANSparkMax(MOTOR_PORT, MotorType.kBrushless);
    }

    public void acquire() {
        motor.set(ACQUIRE_SPEED * 1);
    }

    public void deacquire() {
        motor.set(ACQUIRE_SPEED * -1);
    }

    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Speed", motor.get());
        SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());
    }
}
