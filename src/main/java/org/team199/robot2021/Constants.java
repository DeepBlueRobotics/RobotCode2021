/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Joystick;
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
    public static final double g = 9.81; //meters per second squared
    /**
     * Constants for motor ports and IDs, solenoid ports, sensor numbers, etc.
     * Analagous to RobotMap.java in previous years.
     *
     * Put other constants (like motor speed and characterization constants) in the
     * subsystems themselves.
     */
    public static final class DriveConstants {
        public static final double wheelBase = Units.inchesToMeters(25.0);
        public static final double trackWidth = Units.inchesToMeters(21.5);
        // "swerveRadius" is the distance from the center of the robot to one of the modules
        public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
        // The gearing reduction from the drive motor controller to the wheels
        // "Fast" gearing for the MK3 Swerve Module is 6.86 : 1
        public static final double driveGearing = 6.86;

        public static final double driveModifier = 1;
        public static final double wheelDiameter = Units.inchesToMeters(4.0)*7.36/7.65;
        public static final double mu = 0.15;

        public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
        // Angular speed to translational speed --> v = omega * r / gearing
        public static final double maxSpeed = NEOFreeSpeed * (wheelDiameter / 2.0) / driveGearing * 0.7;
        public static final double maxForward = maxSpeed;
        public static final double maxStrafe = maxSpeed;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / swerveRadius;

        public static final boolean[] reversed = {false, false, false, false};
        // Determine correct turnZero constants (FL, FR, BL, BR)
        public static final double[] turnZero = {141.24, -154.69, 173.144, -43.154};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        // Determine correct turn PID constants
        public static final double[] turnkP = {0.005, 0.005, 0.005, 0.005};
        public static final double[] turnkI = {0, 0, 0, 0};
        public static final double[] turnkD = {0, 0, 0, 0};

        // kP is an average of the forward and backward kP values
        // Forward: 1.72, 1.71, 1.92, 1.94
        // Backward: 1.92, 1.92, 2.11, 1.89
        public static final double[] drivekP = {1.82, 1.815, 2.015, 1.915};
        //public static final double[] drivekP = {0, 0, 0, 0};
        public static final double[] drivekI = {0, 0, 0, 0};
        public static final double[] drivekD = {0, 0, 0, 0};
        public static final boolean[] driveInversion = {true, true, true, true};

        public static final double[] kForwardVolts = {0.129, 0.108, 0.14, 0.125};
        public static final double[] kBackwardVolts = {0.115, 0.169, 0.13, 0.148};
        
        //public static final double[] kForwardVolts = {0, 0, 0, 0};
        //public static final double[] kBackwardVolts = {0, 0, 0, 0};
        public static final double[] kForwardVels = {2.910/1.1, 2.970/1.1, 2.890/1.1, 2.930/1.1};
        public static final double[] kBackwardVels = {2.890/1.1, 2.800/1.1, 2.850/1.1, 2.820/1.1};
        public static final double[] kForwardAccels = {0.145, 0.149, 0.192, 0.198};
        public static final double[] kBackwardAccels = {0.192, 0.187, 0.264, 0.176}; 
        //public static final double[] kForwardAccels = {0, 0, 0, 0};
        //public static final double[] kBackwardAccels = {0, 0, 0, 0};

        public static final double autoMaxSpeed = 0.35 * 4.4;  // Meters / second
        public static final double autoMaxAccel = mu * g;  // Meters / seconds^2
        public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
        // The maximum acceleration the robot can achieve is equal to the coefficient of static friction times the gravitational acceleration
        // a = mu * 9.8 m/s^2
        public static final double autoCentripetalAccel = mu * g * 2;

        // PID values are listed in the order kP, kI, and kD
        public static final double[] xPIDController = {4, 0.0, 0.0};
        public static final double[] yPIDController = {4, 0.0, 0.0};
        public static final double[] thetaPIDController = {4, 0.0, 0.0};

        // TODO: Find experimental value for current draw
        public static final double intakeCurrentDraw = 8.0;

        public static final double cameraHeight = Units.inchesToMeters(40.0);
        public static final double cameraMountingAngleDeg = 29.5;
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

        public static final int kIntakeRoller = 13;

        public static final int kPDPCANPort = 0;
        public static final int kIntakeRollerPDP = 3;

        public static final int kFeederEjector = 7;
        public static final int kFeederBelt = 8;

        //public static final int kShooterMaster = 1;     // Left
        //public static final int kShooterSlave = 14;     // Right

        //public static final int kClimberWinch = 9;
        //public static final int kClimberLift = 13;

        // solenoids
        public static final int[] kIntakePistons = { 0, 1, 2, 5 };

        // other
        public static final int kFeederInSensor = 11;
        public static final int kFeederOutSensor = 10;

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
            public static final int port = 2;

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
            public static final int port = 0;
            public static Joystick controller = new Joystick(port);
            public static int X;
            public static int A;
            public static int B;
            public static int Y;
            public static int LB;
            public static int RB;
            public static int LT;
            public static int RT;
            public static int BACK;
            public static int START;
            //TODO: mode button setting to teletop init
            static {
                if (controller.getName().equals("Logitech Dual Action")) {
                    // Buttons and triggers
                    X = 1;
                    A = 2;
                    B = 3;
                    Y = 4;
                    LB = 5;
                    RB = 6;
                    LT = 7;
                    RT = 8;
                    BACK = 9;
                    START = 10;
                } else {
                    // Buttons and triggers for xbox controller
                    X = 3;
                    A = 1;
                    B = 2;
                    Y = 4;
                    LB = 5;
                    RB = 6;
                    LT = 7;
                    RT = 8;
                    BACK = 9;
                    START = 10;    
                }
            }
            //driving
            static final int kSlowMode = RB;

            // climber
            //public static final int kDeployClimberButton = Y;
            //public static final int kRaiseRobotButton = LB;
            //public static final int kAdjustClimberUpButton = 0; // TODO: change button
            //public static final int kAdjustClimberDownButton = 0; // TODO: change button

            // intake/feeder
            public static final int kIntakeButton = X;
            public static final int kOuttakeButton = Y;
            //public static final int kRegurgitateButton = B;
        }
    }

    public static final class GameConstants {
        //TODO: find actual value
        public static final double[] GSMidPoints = {6, -12, 10};
        
        public static final String[] GSPaths = {"PathARed", "PathBBlue", "PathABlue", "PathBRed"};
    }
}
