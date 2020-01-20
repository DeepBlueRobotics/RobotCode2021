/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Constants for things on our robot. Used to go in RobotMap.java
     */
    public static final class Drive {
        public static final int LEFT_MOTOR_1 = 3;
        public static final int LEFT_MOTOR_2 = 4;
        public static final int RIGHT_MOTOR_1 = 5;
        public static final int RIGHT_MOTOR_2 = 6;

        public static final int[] LEFT_ENCODER = { 0, 1 };
        public static final int[] RIGHT_ENCODER = { 2, 3 };
    }

    /**
     * Constants for controllers and joysticks. Used to go in OI.java
     */
    public static final class OI {
        public static final class LeftJoy {
            public static final int PORT = 1;

            public static final int ARCADETANK_DRIVE_BUTTON = 4;
            public static final int CHARACTERIZED_DRIVE_BUTTON = 5;
            public static final int SLOW_DRIVE_BUTTON = 2;
        }

        public static final class RightJoy {
            public static final int PORT = 0;

            public static final int SLOW_DRIVE_BUTTON = 5;
        }

        public static final class Controller {
            public static final int PORT = 2;
        }
    }

    /**
     * Drivetrain constants used in characterization
     */
    public static final class Drivetrain {
        public static final double[] kVOLTS = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kVELS = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kACCELS = {0.0, 0.0, 0.0, 0.0};
        public static final double MAX_ACCEL = 0.0;
        public static final double MAX_SPEED = 13 * 12; // TODO: change to correct values
        public static final double LOOP_TIME = 0.02;
    }

    public static final double SLOW_DRIVE_SPEED = 0.6;
    public static final double SLOW_DRIVE_ROTATION = 0.6;
}
