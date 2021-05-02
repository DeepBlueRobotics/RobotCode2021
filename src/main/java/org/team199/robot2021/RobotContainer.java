
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.team199.lib.RobotPath;
import org.team199.robot2021.Constants.OI;
import org.team199.robot2021.commands.GalacticSearchCommand;
import org.team199.robot2021.commands.TeleopDrive;
import org.team199.robot2021.commands.ToggleIntake;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import frc.robot.lib.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    final Drivetrain drivetrain = new Drivetrain();
    private final Limelight lime = new Limelight();
    //private final Shooter shooter = new Shooter(lime);
    private final Intake intake = new Intake();
    //private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.port);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.port);
    private final Joystick controller = new Joystick(Constants.OI.Controller.port);
    //private final Climber climber = new Climber();
    private final SendableChooser<Command> autoCommandChooser;
    public Trajectory trajectory;
    public PowerDistributionPanel pdp = new PowerDistributionPanel();
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

        () -> inputProcessing(getStickValue(Constants.OI.StickType.RIGHT, Constants.OI.StickDirection.Y)),
            () -> inputProcessing(getStickValue(Constants.OI.StickType.RIGHT, Constants.OI.StickDirection.X)),
            () -> inputProcessing(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.X))));

        /*
        feeder.setDefaultCommand(new RunCommand(() -> {
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

        autoCommandChooser = new SendableChooser<Command>();
        autoCommandChooser.setDefaultOption("No autonomous", new InstantCommand());

        /* Load AutoNav paths and specify every region where we need to slow down for each path */

        double curvatureRadius = 1;
        // Barrel Racing
        List<EllipticalRegionConstraint>  barrelRegions = new ArrayList<EllipticalRegionConstraint>();
        // Add circular regions around each of the loops around the markers
        barrelRegions.add(RobotPath.createRegionConstraint(3.8, -3, 1, 1, 0, curvatureRadius));
        barrelRegions.add(RobotPath.createRegionConstraint(6.1, -1.5, 1, 1, 0, curvatureRadius));
        barrelRegions.add(RobotPath.createRegionConstraint(7.6, -3, 1, 1, 0, curvatureRadius));
        loadPath("AutoNav: Barrel Racing", "barrelRacing", false, false, false, true, Constants.DriveConstants.autoMaxSpeed, barrelRegions);
        
        // Slalom
        List<EllipticalRegionConstraint>  slalomRegions = new ArrayList<EllipticalRegionConstraint>();
        // Add elliptical regions around where we weave between markers
        slalomRegions.add(RobotPath.createRegionConstraint(2.3, -3.1, 0.1, 0.9, 90, curvatureRadius));
        slalomRegions.add(RobotPath.createRegionConstraint(6.8, -3.1, 0.1, 0.9, 90, curvatureRadius));
        // Add a circular region around the end loop
        slalomRegions.add(RobotPath.createRegionConstraint(7.6, -3, 1, 1, 0, curvatureRadius));
        loadPath("AutoNav: Slalom","slalom", false, false, false, false, Constants.DriveConstants.autoMaxSpeed, slalomRegions);
        
        // Bounce
        List<EllipticalRegionConstraint>  bounceRegions = new ArrayList<EllipticalRegionConstraint>();
        // Add elliptical regions around each marker we weave through / get close to 
        bounceRegions.add(RobotPath.createRegionConstraint(2.3, -3, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(3.8, -3, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(5.3, -3, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(6.1, -3, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(7.6, -3, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(1.5, -1.5, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(3, -1.5, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(3.8, -1.5, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(5.3, -1.5, 1, 1, 0, curvatureRadius));
        bounceRegions.add(RobotPath.createRegionConstraint(7.6, -1.5, 1, 1, 0, curvatureRadius));
        // Create a SequentialCommandGroup for the multiple parts of bounce
        try {
            Command bounce1 = new RobotPath("Bounce1", drivetrain, intake, false, false, Constants.DriveConstants.autoMaxSpeed*0, bounceRegions).getPathCommand(false, false);
            Command bounce2 = new RobotPath("Bounce2", drivetrain, intake, false, false, Constants.DriveConstants.autoMaxSpeed*0, bounceRegions).getPathCommand(false, false);
            Command bounce3 = new RobotPath("Bounce3", drivetrain, intake, false, false, Constants.DriveConstants.autoMaxSpeed*0, bounceRegions).getPathCommand(false, false);
            Command bounce4 = new RobotPath("Bounce4", drivetrain, intake, false, false, Constants.DriveConstants.autoMaxSpeed, bounceRegions).getPathCommand(false, true);
            autoCommandChooser.addOption("AutoNav: Bounce", bounce1.andThen(bounce2, bounce3, bounce4));
        } catch (IOException io) {
            System.err.println("Could not create Bounce autonomous command.");
            io.printStackTrace();
        }

        loadPath("Galactic search: Path A Red","PathARed", true, true, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Galactic search: Path B Red","PathBRed", true, true, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Galactic search: Path A Blue","PathABlue", false, true, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Galactic search: Path B Blue","PathBBlue", false, true, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Square", "Square", false, false, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Figure Eight", "Figure8", false, false, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        loadPath("Straight Line Test", "LineTest", false, false, false, true, Constants.DriveConstants.autoMaxSpeed, new ArrayList<EllipticalRegionConstraint>());
        autoCommandChooser.addOption("Galactic Search: Solution 3", new GalacticSearchCommand(drivetrain, intake, lime));
        SmartDashboard.putData(autoCommandChooser);
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
        new JoystickButton(controller, Constants.OI.Controller.A).whenPressed(new InstantCommand(() -> {drivetrain.resetHeading();}));
        new JoystickButton(controller, Constants.OI.Controller.B).whenPressed(new InstantCommand(() -> { SmartDashboard.putBoolean("Field Oriented", !SmartDashboard.getBoolean("Field Oriented", true)); }));
        // Intake toggle button
        new JoystickButton(controller, Constants.OI.Controller.kIntakeButton).whenPressed(new ToggleIntake(intake));

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
        try {
            final Command autoCommand = autoCommandChooser.getSelected();
            if (autoCommand == null) {
                throw new Exception("No path was selected.");
            }
            return autoCommand;
        } catch(final Exception e) {
            e.printStackTrace(System.err);
            return new InstantCommand();
        }
    }

    private void loadPath(final String chooserName, final String pathName, final boolean faceInPathDirection, final boolean deployIntake, 
                          final boolean isInverted, final boolean stopAtEnd, final double endVelocity, final List<EllipticalRegionConstraint> regionConstraints) {
        try {
            RobotPath path = new RobotPath(pathName, drivetrain, intake, deployIntake, isInverted, endVelocity, regionConstraints);
            autoCommandChooser.addOption(chooserName, path.getPathCommand(faceInPathDirection, stopAtEnd));
        } catch(final Exception e) {
            e.printStackTrace(System.err);
        }
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
                if (controller.getName().equals("Logitech Dual Action")) {
                    if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(0);
                    if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(1);
                    if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(2);
                    if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(3);
                } else {
                    if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(0);
                    if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(1);
                    if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return controller.getRawAxis(4);
                    if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -controller.getRawAxis(5);
                }
            default: return 0;
        }
    }

    /**
     * Processes an input from the joystick into a value between -1 and 1
     * @param value     The value to be processed.
     * @return The processed value.
     */
    private double inputProcessing(double value){
        double processedInput;
        //processedInput = (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
        processedInput = Math.copySign(((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2),value);
        return processedInput;
    }

    /*
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