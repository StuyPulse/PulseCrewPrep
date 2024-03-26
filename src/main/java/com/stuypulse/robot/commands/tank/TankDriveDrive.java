package com.stuypulse.robot.commands.tank;

import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.constants.Ports.Gamepad;
import com.stuypulse.robot.subsystems.TankDrive;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TankDriveDrive extends InstantCommand {

    private final TankDrive tank;
    private final XboxController driver;

    private int frontSpeed;
    private int backSpeed;

    private double angle;

    public TankDriveDrive(TankDrive tank, XboxController driver) {
        this.tank = tank;
        this.driver = driver;
    }

    @Override
    public void execute() {

        if ((driver.getLeftTriggerAxis() < 0.55) || (driver.getLeftTriggerAxis() > 0.45)) {
            angle = 0.5;
        } else {
            angle = driver.getLeftTriggerAxis();
        }

        if (driver.getRightBumperPressed()) {frontSpeed = 1;} else {frontSpeed = 0;}
        if (driver.getLeftBumperPressed()) {backSpeed = 1;} else {backSpeed = 0;}

        tank.arcadeDrive(frontSpeed - backSpeed, angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
