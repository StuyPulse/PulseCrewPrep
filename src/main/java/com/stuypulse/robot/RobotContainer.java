/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.shooter.ShootCommand;
import com.stuypulse.robot.commands.shooter.ShootStopCommand;

import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
    public int DRIVER = 0;
    public int OPERATOR = 1;


    // Gamepads
    // public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    // public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    // replace these with non stuylib controllers
    private final XboxController driver = new XboxController(DRIVER);
    private final XboxController operator = new XboxController(OPERATOR);
    
    // Subsystem
    private final Shooter shooter = new Shooter();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        shooter.setDefaultCommand(new ShootStopCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        if (driver.getRightBumperPressed()) {
            new ShootCommand(1000, 1000);
        }
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
