package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveBack extends Command {
    private SwerveDrive swerve;
    private double speed;

    public SwerveDriveBack(double speed) {
        swerve = SwerveDrive.getInstance();
        this.speed = speed;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(-speed, 0), 0);
    }
}
