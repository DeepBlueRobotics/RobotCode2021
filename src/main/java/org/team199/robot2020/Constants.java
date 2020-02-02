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
     * Constants for controllers and joysticks. Used to go in OI.java
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

            public static final int kSlowDriveButton = 5;
        }

        public static final class Controller {
            public static final int kPort = 2;

            public static final int kIntakeButton = 1; // TODO: change to correct button
            public static final int kOuttakeButton = 1; // TODO: change to correct button
        }
    }

    public static final class Drivetrain {
        public static final int kLeftMaster = 3;
        public static final int kLeftSlave = 4;
        public static final int kRightMaster = 5;
        public static final int kRightSlave = 6;

        public static final int[] kLeftEncoder = { 0, 1 };
        public static final int[] kRightEncoder = { 2, 3 };

        public static final double kTrackWidth = 0.6223;
        // 0.183
        public static final double[] kPidLeft = { 5.45, 0.0, 0.0 };
        // 0.278
        public static final double[] kPidRight = { 6.02, 0.0, 0.0 };

        // 0.232, 0.194, 0.229, 0.198
        public static final double[] kVolts = { 0.228, 0.208, 0.213, 0.199 }; // Volts
        // 0.0545, 0.0529, 0.0545, 0.0528
        public static final double[] kVels = { 1.42, 1.35, 1.41, 1.36 }; // Volt * seconds / inch
        // 0.00475, 0.00597, 0.00318, 0.00616
        public static final double[] kAccels = { 0.0985, 0.133, 0.144, 0.146 }; // Volt * seconds^2 / inch
        public static final double kMaxAccel = 200.0; // Inches / seconds^2
        public static final double kMaxSpeed = 5676 * Math.PI * 5 / 6.8 / 60; // Inches / seconds
        public static final double kMaxAngularSpeed = 4 * Math.PI; // Radians / second
        public static final double kLoopTime = 0.02;

        public static final double kSlowDriveSpeed = 0.6;
        public static final double kSlowDriveRotation = 0.6;
    }

    public static final class Intake {
        public static final int kIntakeMotor = 0; // TODO: set all to correct ports
        public static final int kIntakeDownMotor = 1; // TODO: set all to correct ports
        public static final double kIntakeSpeed = 1; // TODO: set correct speeds
    }

    public static final class Feeder {
        // TODO: change to correct values
        public static final int kBeltMotor = 2;
        public static final int kEjectMotor = 3;

        public static final int kIndexSensor = 0;

        public static final double kBeltSpeed = .8;
        public static final double kEjectSpeed = 1;

        public static final double kIndexerDistance = 127; // in mm
    }
}
