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
     * Constants for motor ports and IDs, solenoid ports, sensor numbers, etc.
     * Analagous to RobotMap.java in previous years.
     * 
     * Put other constants (like motor speed and characterization constants) in the
     * subsystems themselves.
     */
    public static final class Drive {
        // CAN IDs
        //TODO: Change to correct ports
        public static final int kDtLeftMaster = 3;
        public static final int kDtLeftSlave = 4;
        public static final int kDtRightMaster = 5;
        public static final int kDtRightSlave = 6;
        public static final int kIntakeRoller = 0;
        public static final int kFeederBelt = 1;
        public static final int kFeederEjector = 2;
        public static final int kShooterVictor = 9;
        public static final int kShooterMaster = 2;
        public static final int kShooterSlave = 11;
        public static final int kClimberWinch = 0; //TODO: Find Ports
        public static final int kClimberLift = 1; //TODO: Find Ports

        // solenoids
        public static final int[] kIntakePistons = { 0, 1 };

        // other
        public static final int kFeederInSensor = 0;
        public static final int kFeederOutSensor = 1;

        public static final int kAutoPathSwitch1Port = 0;
        public static final int kAutoPathSwitch2Port = 1;
    }

    /**
     * Constants for controllers and joysticks. Analagous to OI.java in previous
     * years.
     */
    public static final class OI {
        public static final class LeftJoy {
            public static final int kPort = 0;

            public static final int kToggleDriveModeButton = 4;
            public static final int kCharacterizedDriveButton = 5;
            public static final int kSlowDriveButton = 2;
        }

        public static final class RightJoy {
            public static final int kPort = 1;

            public static final int kAlignAndShootButton = 2;
            public static final int kSlowDriveButton = 5;
        }

        public static final class Controller {
            public static final int kPort = 2;

            // climber
            public static final int kDeployClimberButton = 3; //TODO: Find Ports
            public static final int kRaiseRobotButton = 5; //TODO: Find Ports
            public static final int kAdjustClimberUpButton = 0; // TODO: change button
            public static final int kAdjustClimberDownButton = 0; // TODO: change button    

            // intake/feeder
            public static final int kIntakeButton = 1; // TODO: change to correct button
            public static final int kOuttakeButton = 2; // TODO: change to correct button
            public static final int kRegurgitateButton = 3; // TODO: change to correct button
        }
    }
}
