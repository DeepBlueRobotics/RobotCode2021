
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.team199.lib.RobotPath;
import org.team199.robot2021.Constants.OI;
import org.team199.robot2021.commands.HomeAbsolute;
import org.team199.robot2021.commands.TeleopDrive;
import org.team199.robot2021.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //private final DigitalInput autoSwitch1 = new DigitalInput(Constants.DrivePorts.kAutoPathSwitch1Port);
    //private final DigitalInput autoSwitch2 = new DigitalInput(Constants.DrivePorts.kAutoPathSwitch2Port);
    final Drivetrain drivetrain = new Drivetrain();
    //private final Limelight lime = new Limelight();
    //private final Shooter shooter = new Shooter(lime);
    //private final Intake intake = new Intake();
    //private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.port);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.port);
    private final Joystick controller = new Joystick(Constants.OI.Controller.port);
    //private final Climber climber = new Climber();
    //private final RobotPath[] paths;
    //private final LinearInterpolation linearInterpol;

    public RobotContainer() {

        if(DriverStation.getInstance().getJoystickName(OI.LeftJoy.port).length() != 0) {
            configureButtonBindingsLeftJoy();
        } else{
            System.err.println("ERROR: Dude, you're missing the left joystick.");
        }

        if(DriverStation.getInstance().getJoystickName(OI.RightJoy.port).length() != 0) {
            configureButtonBindingsRightJoy();
        } else{
            System.err.println("ERROR: Dude, you're missing the right joystick.");
        }

        if(DriverStation.getInstance().getJoystickName(OI.Controller.port).length() != 0) {
            configureButtonBindingsController();
        } else{
            System.err.println("ERROR: Dude, you're missing the controller.");
        }

        //shooter.setDefaultCommand(new RunCommand(()-> shooter.setSpeed(shooter.getTargetSpeed()), shooter));
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain,
            () -> signedSquare(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.Y)),
            () -> signedSquare(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.X)),
            () -> signedSquare(getStickValue(Constants.OI.StickType.RIGHT, Constants.OI.StickDirection.X))));


        /*feeder.setDefaultCommand(new RunCommand(() -> {
            if (feeder.isCellEntering() && !feeder.isCellAtShooter()) {
                feeder.runForward();
                if(intake.isDeployed())
                    intake.slow();
            } else {
                feeder.stop();
                if(intake.isDeployed())
                    intake.intake();
            }
        }, feeder, intake));*/

        //paths = new RobotPath[4];
        //if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            /*loadPath(Path.PATH1, "AutoLeft", false, StartingPosition.BLUE_LEFT.pos);
            loadPath(Path.PATH2, "OneBall", false, StartingPosition.BLUE_CENTER.pos);
            loadPath(Path.PATH3, "AutoRight", false, StartingPosition.BLUE_RIGHT.pos);*/
        //} else {
            /*loadPath(Path.PATH1, "AutoLeft", false, StartingPosition.RED_LEFT.pos);
            loadPath(Path.PATH2, "OneBall", false, StartingPosition.RED_CENTER.pos);
            loadPath(Path.PATH3, "AutoRight", false, StartingPosition.RED_RIGHT.pos);*/
        //}
        //linearInterpol = new LinearInterpolation("ShooterData.csv");
    }

    private void configureButtonBindingsLeftJoy() {
        // Arcade/Tank drive button
        //new JoystickButton(leftJoy, Constants.OI.LeftJoy.kToggleDriveModeButton).whenPressed(new InstantCommand(
        //        () -> SmartDashboard.putBoolean("Arcade Drive", !SmartDashboard.getBoolean("Arcade Drive", false))));

        // characterize drive button

        // Toggle Characterize Drive
        //new JoystickButton(leftJoy, Constants.OI.LeftJoy.kCharacterizedDriveButton).whenPressed(new InstantCommand(
        //        () -> SmartDashboard.putBoolean("Characterized Drive", !SmartDashboard.getBoolean("Characterized Drive", false))));
    }

    private void configureButtonBindingsRightJoy() {
        new JoystickButton(rightJoy, 3).whenPressed(new InstantCommand(drivetrain::toggleMode, drivetrain));
        // Align the robot and then shoots
        //new JoystickButton(rightJoy, Constants.OI.RightJoy.kAlignAndShootButton).whileHeld(new SequentialCommandGroup(new ShooterHorizontalAim(drivetrain, lime), new Shoot(feeder)));
    }

    private void configureButtonBindingsController() {
        new JoystickButton(controller, Constants.OI.Controller.A).whenPressed(new HomeAbsolute(drivetrain));
        new JoystickButton(controller, Constants.OI.Controller.B).whenPressed(new InstantCommand(() -> { SmartDashboard.putBoolean("Field Oriented", !SmartDashboard.getBoolean("Field Oriented", true)); }));
        // Intake toggle button
        /*new JoystickButton(controller, Constants.OI.Controller.kIntakeButton).whenPressed(new InstantCommand(() -> {
            if (intake.isDeployed()) {
                intake.retract();
                intake.stop();
            } else {
                intake.doTheFlop();
                intake.intake();
            }
        }, intake));*/

        // Power cell regurgitate button
        //new JoystickButton(controller, Constants.OI.Controller.kRegurgitateButton).whileHeld(new Regurgitate(intake, feeder));

        // Deploy climber button and allow for adjustment
        /*new JoystickButton(controller, Constants.OI.Controller.kDeployClimberButton).whenPressed(new SequentialCommandGroup(
            new DeployClimber(climber),
            new AdjustClimber(climber, controller)
        ));*/

        // climb button
        //new JoystickButton(controller, Constants.OI.Controller.kRaiseRobotButton).whenPressed(new RaiseRobot(climber));
    }

    public Command getAutonomousCommand() {
        /*
        try {
            final RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            }
            boolean isBlue = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue;
            return new AutoShootAndDrive(drivetrain, intake, feeder,
                                         shooter, lime, path,
                                         linearInterpol, (isBlue ? Target.BLUE_PORT.pos : Target.RED_PORT.pos));
        } catch(final Exception e) {
            return new InstantCommand();
        }*/
        return null;
    }

    /**
     * Get the stick value of a joystick given its stick type (left side or right side) and its axis (X or Y).
     * @param stick   The stick type of the joystick, either LEFT for left joystick or RIGHT for right joystick.
     * @param dir     The direction, or axis, of the joystick, either X for the x-axis or Y for the y-axis.
     * @return A double representing how far the joystick has been pushed, between -1.0 (all the way backwards) to 1.0 (all the way forwards).
     */
    private double getStickValue(Constants.OI.StickType stick, Constants.OI.StickDirection dir) {
        switch (Constants.OI.CONTROL_TYPE) {
            case JOYSTICKS:
                if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return leftJoy.getX();
                if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -leftJoy.getY();
                if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return rightJoy.getX();
                if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -rightJoy.getY();
            case GAMEPAD:
                if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(0);
                if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(1);
                if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(2);
                if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(3);
            default: return 0;
        }
    }

    /**
     * Squares a value and preserves its sign.
     * @param value     The value to be squared.
     * @return The signed squared value.
     */
    private double signedSquare(double value){
        return value * Math.abs(value);
    }

    /**
     * DIO Port 0 = Switch 1
     * DIO Port 1 = Switch 2
     * on = jumper in
     * off= jumper out
     * Red/Blue determined by DS
     * Switch states
     * 1    2
     * off off = off
     * on off = 1
     * off on = 2
     * on on = 3
     */
     /*
    public Path getPath() {
        Path outPath = Path.OFF;
        // get() returns true if the circuit is open.
        if(!autoSwitch1.get()) {
            if(!autoSwitch2.get()) {
                outPath = Path.PATH3;
                System.out.println("Path3 loaded.");
            } else {
                outPath = Path.PATH1;
                System.out.println("Path1 loaded.");
            }
        } else if(!autoSwitch2.get()) {
            outPath = Path.PATH2;
            System.out.println("Path2 loaded.");
        } else {
            outPath = Path.OFF;
            System.out.println("No path loaded.");
        }
        return outPath;
    }

    private void loadPath(final Path path, final String pathName, final boolean isInverted, final Translation2d initPos) {
        try {
            paths[path.idx] = new RobotPath(pathName, drivetrain, isInverted, initPos);
        } catch(final Exception e) {
            System.err.println("Error Occured Loading Path: [" + path.name() + "," + pathName + "]");
            e.printStackTrace(System.err);
        }
    }

    public static enum Path {
        PATH1(0), PATH2(1), PATH3(2), OFF(-1);

        public final int idx;

        private Path(final int idx) {
            this.idx = idx;
        }
    }

    public static enum StartingPosition {
        // TODO: Change starting positions to reflect FIRST At Home challenges
        BLUE_LEFT(12.61, -4.75),
        BLUE_CENTER(12.61, -5.75),
        BLUE_RIGHT(12.61, -6.75),
        RED_LEFT(3.39, -3.4),
        RED_CENTER(3.39, -2.4),
        RED_RIGHT(3.39, -1.4);

        public final Translation2d pos;

        private StartingPosition(double x, double y) {
            pos = new Translation2d(x, y);
        }
    }
    public static enum Target {
        BLUE_PORT(16, -5.75),
        RED_PORT(0, -2.4);

        public final Translation2d pos;

        private Target(double x, double y) {
            pos = new Translation2d(x, y);
        }
      }
    */
}
