package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {

    private final Shooter shooter;

    public ShootCommand() {
        shooter = new Shooter();

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot();
    }

}
