/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

     Boolean DEBUG_MODE = true;

    public interface TankDrive {
        // If speed is below this, use quick turn
        
        double BASE_TURNING_SPEED =  0.45;

        // Low Pass Filter and deadband for Driver Controls
       double SPEED_DEADBAND = 0.00;
       double ANGLE_DEADBAND = 0.00;

        double SPEED_POWER = 2.0;
        double ANGLE_POWER =  1.0;

       double SPEED_FILTER = 0.125;
       double ANGLE_FILTER = 0.005;


        boolean USING_GYRO = true;

        // Width of the robot
        // update
        double TRACK_WIDTH = Units.inchesToMeters(26.9);

    }
        public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(13.0);

        double MAX_MODULE_SPEED = 4.9;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        double ALIGN_OMEGA_DEADBAND =  0.05; //"Swerve/Align Omega Deadband"

        
        
        public interface Driver {


            public interface Drive {
                double kP = 0.31399; //"Swerve/Drive/PID/kP"
                double kI = 0.0; 
                double kD = 0.0;

                double kS = 0.27354;
                double kV = 2.1022; //"Swerve/Drive/FF/kV"
                double kA = 0.41251; //"Swerve/Drive/FF/kA"
            }
            
    
            public interface Turn {
                double DEADBAND = 0.03; //"Driver Settings/Turn/Deadband"
    
                double RC = 0.05; //"Driver Settings/Turn/RC"
                double POWER = 2; //"Driver Settings/Turn/Power"
    
                double MAX_TELEOP_TURNING = 6.0; //"Driver Settings/Turn/Max Turning"
            }
        }
    }
}
