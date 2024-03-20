package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {

    private final Shooter shooter;
    private final double leftRPM; 
    private final double rightRPM;

    public ShootCommand(double leftRPM, double rightRPM) {
        shooter = new Shooter();
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(leftRPM, rightRPM);
    }

}
