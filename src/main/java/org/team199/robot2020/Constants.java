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
            public static final int PORT = 0;

            public static final int ARCADETANK_DRIVE_BUTTON = 4;
            public static final int CHARACTERIZED_DRIVE_BUTTON = 5;
            public static final int SLOW_DRIVE_BUTTON = 2;
        }

        public static final class RightJoy {
            public static final int PORT = 1;

            public static final int SLOW_DRIVE_BUTTON = 5;
        }

        public static final class Controller {
            public static final int PORT = 2;
            public static final int DEPLOY_CLIMBER_BUTTON = 3; //TODO: Find Ports
            public static final int RAISE_ROBOT_BUTTON = 5; //TODO: Find Ports

            public static final int ADJUST_CLIMBER_UP_BUTTON = 0; // TODO: change button
            public static final int ADJUST_CLIMBER_DOWN_BUTTON = 0; // TODO: change button    
        }
    }

    /**
     * Drivetrain constants used in characterization
     */
    public static final class Drivetrain {
        public static final double TRACKWIDTH = 0.6223;
        // 0.183
        public static final double[] kPIDLEFT = {5.45, 0.0, 0.0};
        // 0.278
        public static final double[] kPIDRIGHT = {6.02, 0.0, 0.0};

        // 0.232, 0.194, 0.229, 0.198
        public static final double[] kVOLTS = {0.228, 0.208, 0.213, 0.199};  // Volts
        // 0.0545, 0.0529, 0.0545, 0.0528
        public static final double[] kVELS = {1.42, 1.35, 1.41, 1.36};  // Volt * seconds / inch
        // 0.00475, 0.00597, 0.00318, 0.00616
        public static final double[] kACCELS = {0.0985, 0.133, 0.144, 0.146};  // Volt * seconds^2 / inch
        public static final double MAX_ACCEL = 200.0;  // Inches / seconds^2
        public static final double MAX_SPEED = 5676 * Math.PI * 5 / 6.8 / 60; // Inches / seconds
        public static final double MAX_ANGULAR_SPEED = 4 * Math.PI; // Radians / second
        public static final double LOOP_TIME = 0.02;
    }

    public static final class Climber {
        public static final int WINCH_MOTOR = 0; //TODO: Find Ports
        public static final int LIFT_MOTOR = 1; //TODO: Find Ports

        public static final double LIFT_DEPLOY_SPEED = 0.9; // TODO: set correct speed
        public static final double WINCH_DEPLOY_SPEED = 0.5; // TODO: set correct speed
        public static final double LIFT_KEEP_SPEED = 0.2; // TODO: set correct speed
        public static final double LIFT_RETRACT_SPEED = -0.5; // TODO: set correct speed
        public static final double WINCH_RETRACT_SPEED = 0.8; // TODO: set correct speed
        public static final double LIFT_ADJUST_SPEED = 0.1; // TODO: set correct speed

        public static final double LIFT_HEIGHT = 18.8;
        public static final double WINCH_START_HEIGHT = -90;
        public static final double WINCH_END_HEIGHT = 76.2;

        public static final double LIFT_CONVERSION_FACTOR = 43.98 / 256; // TODO: confirm numbers (inches)
        public static final double WINCH_CONVERSION_FACTOR = 26.39 / 256; // TODO: confirm numbers (inches)

    }

    public static final double SLOW_DRIVE_SPEED = 0.6;
    public static final double SLOW_DRIVE_ROTATION = 0.6;
}
