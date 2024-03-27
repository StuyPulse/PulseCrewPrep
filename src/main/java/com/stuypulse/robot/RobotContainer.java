/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.PreloadMobility;
import com.stuypulse.robot.commands.tank.TankDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final XboxController driver = new XboxController(Ports.Gamepad.DRIVER);
    // public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    // replace these with non stuylib controllers
    
    // Subsystem
    public TankDrive drivetrain = TankDrive.getInstance();

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
        drivetrain.setDefaultCommand(new TankDriveDrive(drivetrain, driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Preload Mobility", new PreloadMobility());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
