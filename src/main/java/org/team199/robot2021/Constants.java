/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;
import edu.wpi.first.wpilibj.util.Units;

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
    public static final class DriveConstants {
        // TODO: Determine more exact measurements for wheel base and track width
        public static final double wheelBase = Units.inchesToMeters(23.5);
        public static final double trackWidth = Units.inchesToMeters(20.5);
        // The gearing reduction from the drive motor controller to the wheels
        // "Fast" gearing for the MK3 Swerve Module is 6.86 : 1
        public static final double driveGearing = 6.86;

        public static final double driveModifier = 1;
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double mu = 0.6;

        public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
        // Angular speed to translational speed --> v = omega * r / gearing
        public static final double maxSpeed = NEOFreeSpeed * (wheelDiameter / 2.0) / driveGearing;
        public static final double maxForward = maxSpeed;
        public static final double maxStrafe = maxSpeed;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));

        public static final boolean[] reversed = {true, true, true, true};
        // TODO: Determine correct turnZero constants
        public static final int[] turnZero = {140, -154, 54, -45};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        // TODO: Determine correct turn PID constants
        public static final double[] turnkP = {0.01, 0.01, 0.01, 0.01};
        public static final double[] turnkI = {0, 0, 0, 0};
        public static final double[] turnkD = {0, 0, 0, 0};

        public static final double[] drivekP = {0, 0, 0, 0};
        public static final double[] drivekI = {0, 0, 0, 0};
        public static final double[] drivekD = {0, 0, 0, 0};

        public static final double[] kForwardVolts = {0, 0, 0, 0.0};
        public static final double[] kBackwardVolts = {0, 0, 0, 0};
        public static final double[] kForwardVels = {1.0/maxSpeed, 1.0/maxSpeed, 1.0/maxSpeed, 1.0/maxSpeed};
        public static final double[] kBackwardVels = {1.0/maxSpeed, 1.0/maxSpeed, 1.0/maxSpeed, 1.0/maxSpeed};
        public static final double[] kForwardAccels = {0, 0, 0, 0};
        public static final double[] kBackwardAccels = {0, 0, 0, 0};

        public static final double autoMaxSpeed = 1.0;  // Meters / second
        public static final double autoMaxAccel = 0.847;  // Meters / seconds^2
        public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
    }

    public static final class DrivePorts {
        // CAN ids for each of the drive motor controllers.
        public static final int driveFrontLeft = 9;
        public static final int driveFrontRight = 7;
        public static final int driveBackLeft = 4;
        public static final int driveBackRight = 2;

        // CAN ids for each of the turn motor controllers.
        public static final int turnFrontLeft = 1;
        public static final int turnFrontRight = 11;
        public static final int turnBackLeft = 3;
        public static final int turnBackRight = 5;

        // CAN ids for each of the CANCoders
        // TODO: figure out correct ports
        public static final int canCoderPortFL = 4;
        public static final int canCoderPortFR = 3;
        public static final int canCoderPortBL = 1;
        public static final int canCoderPortBR = 2;

        //public static final int kIntakeRoller = 2;

        //public static final int kFeederEjector = 7;
        //public static final int kFeederBelt = 8;

        //public static final int kShooterMaster = 1;     // Left
        //public static final int kShooterSlave = 14;     // Right

        //public static final int kClimberWinch = 9;
        //public static final int kClimberLift = 13;

        // solenoids
        //public static final int[] kIntakePistons = { 0, 1, 2, 5 };

        // other
        //public static final int kFeederInSensor = 11;
        //public static final int kFeederOutSensor = 10;

        public static final int kAutoPathSwitch1Port = 0;
        public static final int kAutoPathSwitch2Port = 1;
    }

    /**
     * Constants for controllers and joysticks. Analagous to OI.java in previous
     * years.
     */
    public static final class OI {
        // For determining outputs of joysticks.
        public static enum ControlType {JOYSTICKS, GAMEPAD};
        public static enum StickType {LEFT, RIGHT};
        public static enum StickDirection {X, Y};

        public static ControlType CONTROL_TYPE = ControlType.GAMEPAD;
        public static final double JOY_THRESH = 0.01;
        public static final class LeftJoy {
            public static final int port = 0;

            //public static final int toggleDriveModeButton = 4;
            //public static final int characterizedDriveButton = 5;
            //public static final int slowDriveButton = 2;
        }

        public static final class RightJoy {
            public static final int port = 1;

            //public static final int alignAndShootButton = 2;
            //public static final int slowDriveButton = 5;
        }

        public static final class Controller {
            public static final int port = 2;

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
            //public static final int kDeployClimberButton = Y;
            //public static final int kRaiseRobotButton = LB;
            //public static final int kAdjustClimberUpButton = 0; // TODO: change button
            //public static final int kAdjustClimberDownButton = 0; // TODO: change button

            // intake/feeder
            //public static final int kIntakeButton = X;
            //public static final int kOuttakeButton = A;
            //public static final int kRegurgitateButton = B;
        }
    }
}
