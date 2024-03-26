package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAcquire extends Command{
    private Intake intake;

    public IntakeAcquire() {
        intake = intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

}
