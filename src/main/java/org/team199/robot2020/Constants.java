/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import edu.wpi.first.wpilibj.geometry.Translation2d;

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
    public static final class Ports {
        // CAN IDs
        public static final int kPDPCanID = 0;

        public static final int kDtLeftMaster = 3;
        public static final int kDtLeftSlave = 4;
        public static final int kDtRightMaster = 5;
        public static final int kDtRightSlave = 6;

        public static final int kIntakeRoller = 2;

        public static final int kFeederEjector = 7;
        public static final int kFeederBelt = 8;

        public static final int kShooterMaster = 1;     // Left
        public static final int kShooterSlave = 14;     // Right

        public static final int kClimberWinch = 9;
        public static final int kClimberLift = 13;

        // solenoids
        public static final int[] kIntakePistons = { 0, 1, 2, 5 };

        // other
        public static final int kFeederInSensor = 11;
        public static final int kFeederOutSensor = 10;

        public static final int kAutoPathSwitch1Port = 0;
        public static final int kAutoPathSwitch2Port = 1;
        public static final int kAutoPathSwitch3Port = 2;

        // PDP ports
        public static final int kFeederBeltPDP = 4;
        public static final int kFeederEjectorPDP = 5;

        public static final int kCamera1Port = 0;
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
            // camera
            public static final int kToggleCameraButton = 3;
        }

        public static final class RightJoy {
            public static final int kPort = 1;

            public static final int kAlignAndShootButton = 2;
            public static final int kShootButton = 3;
            public static final int kSlowDriveButton = 5;
            public static final int kToggleBreakModeButton = 4;
            public static final int kUserCommandConfirmButton = 6;
        }

        public static final class Controller {
            public static final int kPort = 2;

            // Buttons and triggers
            public static final int X = 1;
            public static final int A = 2;
            public static final int B = 3;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int LT = 7;
            public static final int RT = 8;
            public static final int BACK = 9;
            public static final int START = 10;

            // climber
            public static final int kDeployClimberButton = BACK;
            public static final int kRaiseRobotButton = START;
            public static final int kAdjustClimberUpButton = LB;
            public static final int kAdjustClimberDownButton = RB;

            // intake/feeder
            public static final int kIntakeButton = X;
            public static final int kOuttakeButton = A;
            public static final int kRegurgitateButton = B;

            public static final int kSetOdometyButton = LT;
            public static final int kResetOdometyButton = RT;

        }
    }

    public static enum FieldPositions {
        // DO NOT CHANGE ANY OF THESE VALUES.
        BLUE_LEFT(12.61, -4.75), 
        BLUE_CENTER(12.61, -5.75), 
        BLUE_RIGHT(12.61, -6.75),
        BLUE_GS(12.61, -5.75), 
        RED_LEFT(3.39, -3.4), 
        RED_CENTER(3.39, -2.4), 
        RED_RIGHT(3.39, -1.4),
        RED_GS(3.39, -5.6),
        BLUE_PORT(16, -5.75), 
        RED_PORT(0, -2.4);

        public final Translation2d pos;

        private FieldPositions(double x, double y) {
            pos = new Translation2d(x, y);
        }
    }
}
