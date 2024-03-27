package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.SwerveDriveBack;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Mobility extends SequentialCommandGroup {
    public Mobility() {
        addCommands(
            new WaitCommand(8),
            new SwerveDriveBack(2).withTimeout(2)
        );
    }
}
