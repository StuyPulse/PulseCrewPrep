package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class PreloadMobility extends SequentialCommandGroup {

    public PreloadMobility() {
        addCommands(
            new Preload(),
            new Mobility()
        );
    }
}
