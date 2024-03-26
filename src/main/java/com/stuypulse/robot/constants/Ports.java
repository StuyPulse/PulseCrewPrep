/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface TankDrive{
        int LEFT_TOP = 0;
        int LEFT_MIDDLE = 1;
        int LEFT_BOTTOM = 2;

        int RIGHT_TOP = 3;
        int RIGHT_MIDDLE = 4;
        int RIGHT_BOTTOM = 5;
    }

    interface Grayhill {
        // Sensors
        int LEFT_A = 0;
        int LEFT_B = 1;

        int RIGHT_A = 2;
        int RIGHT_B = 3;
    }

}
