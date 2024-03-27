package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.shooter.ShootCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Preload extends SequentialCommandGroup {
    public Preload() {
        addCommands(
            new WaitCommand(2),
            new ShootCommand(1,1)
        );
    }
}
