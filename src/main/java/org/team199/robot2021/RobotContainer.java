
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.lib.Limelight;
import frc.robot.lib.LinearInterpolation;
import org.team199.lib.RobotPath;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team199.robot2021.commands.Regurgitate;
import org.team199.robot2021.commands.TeleopDrive;
import org.team199.robot2021.commands.Shoot;
import org.team199.robot2021.commands.ShooterHorizontalAim;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Shooter;
import org.team199.robot2021.subsystems.Turret;
import org.team199.robot2021.commands.AdjustClimber;
import org.team199.robot2021.commands.AutoShootAndDrive;
import org.team199.robot2021.commands.CalibrateTurret;
import org.team199.robot2021.commands.DeployClimber;
import org.team199.robot2021.commands.RaiseRobot;
import org.team199.robot2021.subsystems.Feeder;
import org.team199.robot2021.subsystems.Intake;
import org.team199.robot2021.subsystems.Climber;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final DigitalInput autoSwitch1 = new DigitalInput(Constants.Drive.kAutoPathSwitch1Port);
    private final DigitalInput autoSwitch2 = new DigitalInput(Constants.Drive.kAutoPathSwitch2Port);
    final Drivetrain drivetrain = new Drivetrain();
    private final Limelight lime = new Limelight();
    private final Shooter shooter = new Shooter(lime);
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.kPort);
    private final Joystick controller = new Joystick(Constants.OI.Controller.kPort);
    private final Climber climber = new Climber();
    private final Turret turret = new Turret();
    private final RobotPath[] paths;
    private final LinearInterpolation linearInterpol;

    public RobotContainer() {
        
        if(DriverStation.getInstance().getJoystickName(0).length() != 0) {
            configureButtonBindingsLeftJoy();
        } else{
            System.err.println("ERROR: Dude, you're missing the left joystick.");
        }

        if(DriverStation.getInstance().getJoystickName(1).length() != 0) {
            configureButtonBindingsRightJoy();
        } else{
            System.err.println("ERROR: Dude, you're missing the right joystick.");
        }

        if(DriverStation.getInstance().getJoystickName(2).length() != 0) {
            configureButtonBindingsController();
        } else{
            System.err.println("ERROR: Dude, you're missing the controller.");
        }

        shooter.setDefaultCommand(new RunCommand(()-> shooter.setSpeed(shooter.getTargetSpeed()), shooter));
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, leftJoy, rightJoy, lime));
        
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
        }, feeder, intake));

        turret.setDefaultCommand(new RunCommand(turret::stop, turret));

        paths = new RobotPath[4];
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            loadPath(Path.PATH1, "AutoLeft", false, StartingPosition.BLUE_LEFT.pos);
            loadPath(Path.PATH2, "OneBall", false, StartingPosition.BLUE_CENTER.pos);
            loadPath(Path.PATH3, "AutoRight", false, StartingPosition.BLUE_RIGHT.pos);
        } else {
            loadPath(Path.PATH1, "AutoLeft", false, StartingPosition.RED_LEFT.pos);
            loadPath(Path.PATH2, "OneBall", false, StartingPosition.RED_CENTER.pos);
            loadPath(Path.PATH3, "AutoRight", false, StartingPosition.RED_RIGHT.pos);
        }
        linearInterpol = new LinearInterpolation("ShooterData.csv");
    }

    private void configureButtonBindingsLeftJoy() {
        // Arcade/Tank drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kToggleDriveModeButton).whenPressed(new InstantCommand(
                () -> SmartDashboard.putBoolean("Arcade Drive", !SmartDashboard.getBoolean("Arcade Drive", false))));

        // characterize drive button
        
        // Toggle Characterize Drive                
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kCharacterizedDriveButton).whenPressed(new InstantCommand(
                () -> SmartDashboard.putBoolean("Characterized Drive", !SmartDashboard.getBoolean("Characterized Drive", false))));
    }

    private void configureButtonBindingsRightJoy() {
        new JoystickButton(rightJoy, 3).whenPressed(new InstantCommand(drivetrain::toggleMode, drivetrain));
        // Align the robot and then shoots
        new JoystickButton(rightJoy, Constants.OI.RightJoy.kAlignAndShootButton).whileHeld(new SequentialCommandGroup(new ShooterHorizontalAim(turret, lime), new Shoot(feeder)));
    }

    private void configureButtonBindingsController() {
        // Intake toggle button
        new JoystickButton(controller, Constants.OI.Controller.kIntakeButton).whenPressed(new InstantCommand(() -> {
            if (intake.isDeployed()) {
                intake.retract();
                intake.stop();
            } else {
                intake.doTheFlop();
                intake.intake();
            }
        }, intake));

        // Power cell regurgitate button
        new JoystickButton(controller, Constants.OI.Controller.kRegurgitateButton).whileHeld(new Regurgitate(intake, feeder));

        // Deploy climber button and allow for adjustment
        new JoystickButton(controller, Constants.OI.Controller.kDeployClimberButton).whenPressed(new SequentialCommandGroup(
            new DeployClimber(climber),
            new AdjustClimber(climber, controller)
        ));

        // climb button
        new JoystickButton(controller, Constants.OI.Controller.kRaiseRobotButton).whenPressed(new RaiseRobot(climber));

        // turn turret
        new JoystickButton(controller, Constants.OI.Controller.kTurnTurretCounterclockwiseButton).whileHeld(new InstantCommand(turret::turnCounterclockwise, turret));
        new JoystickButton(controller, Constants.OI.Controller.kTurnTurretClockwiseButton).whileHeld(new InstantCommand(turret::turnClockwise, turret));

        // calibrate turret
        new JoystickButton(controller, Constants.OI.Controller.kCalibrateTurret).whenPressed(new CalibrateTurret(turret));
    }

    public Command getAutonomousCommand() {
        try {
            final RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            }
            boolean isBlue = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue;
            return new AutoShootAndDrive(drivetrain, intake, feeder, 
                                         shooter, turret, lime, path, 
                                         linearInterpol, (isBlue ? Target.BLUE_PORT.pos : Target.RED_PORT.pos));
        } catch(final Exception e) {
            return new InstantCommand();
        }
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
        // DO NOT CHANGE ANY OF THESE VALUES.
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
        // DO NOT CHANGE ANY OF THESE VALUES.
        BLUE_PORT(16, -5.75), 
        RED_PORT(0, -2.4); 
    
        public final Translation2d pos;
    
        private Target(double x, double y) {
            pos = new Translation2d(x, y);
        }
      }
}
