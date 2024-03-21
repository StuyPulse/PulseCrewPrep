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
    
}
