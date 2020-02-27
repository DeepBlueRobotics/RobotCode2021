
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import org.team199.lib.Limelight;

import org.team199.lib.RobotPath;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team199.robot2020.commands.Regurgitate;
import org.team199.robot2020.commands.TeleopDrive;
import org.team199.robot2020.commands.Shoot;
import org.team199.robot2020.commands.ShooterHorizontalAim;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Shooter;
import org.team199.robot2020.commands.AdjustClimber;
import org.team199.robot2020.commands.AutoShootAndDrive;
import org.team199.robot2020.commands.DeployClimber;
import org.team199.robot2020.commands.RaiseRobot;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;
import org.team199.robot2020.subsystems.Climber;

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
    private final RobotPath[] paths;


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

        //1,6 2,5
        //DoubleSolenoid sol1 = new DoubleSolenoid(1,6);
        //DoubleSolenoid sol2 = new DoubleSolenoid(2,5);
        //sol1.set(DoubleSolenoid.Value.kOff);
        //sol2.set(DoubleSolenoid.Value.kOff);

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

        paths = new RobotPath[6];
        loadPath(Path.BLUE1, "Blue1", false);
        loadPath(Path.BLUE2, "TestBlue2OneBall", false);
        loadPath(Path.BLUE3, "Blue3", false);
        loadPath(Path.RED1, "Red1", false);
        loadPath(Path.RED2, "Red2", false);
        loadPath(Path.RED3, "Red3", false);
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

    private void configureButtonBindingsRightJoy() {new JoystickButton(rightJoy, 3).whenPressed(new InstantCommand(drivetrain::toggleMode, drivetrain));
        // Align the robot and then shoots
        new JoystickButton(rightJoy, Constants.OI.RightJoy.kAlignAndShootButton).whileHeld(new SequentialCommandGroup(new ShooterHorizontalAim(drivetrain, lime), new Shoot(feeder)));
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
    }

    public Command getAutonomousCommand() {
        try {
            final RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            } return new AutoShootAndDrive(drivetrain, intake, feeder, shooter, lime, path);
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
                outPath = Path.BLUE3;
                System.out.println("Blue3 loaded.");
            } else {
                outPath = Path.BLUE1;
                System.out.println("Blue1 loaded.");
            }
        } else if(!autoSwitch2.get()) {
            outPath = Path.BLUE2;
            System.out.println("Blue2 loaded.");
        } else {
            outPath = Path.OFF;
            System.out.println("No path loaded.");
        }
        return outPath.toSide(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);
    }

    private void loadDPath(Path path, String pathName, boolean isInverted) {
        loadPath(path.toSide(true), pathName, isInverted);
        loadPath(path.toSide(false), pathName, !isInverted);
    }
        
    private void loadPath(final Path path, final String pathName, final boolean isInverted) {
        try {
            paths[path.idx] = new RobotPath(pathName, drivetrain, isInverted);
        } catch(final Exception e) {
            System.err.println("Error Occured Loading Path: [" + path.name() + "," + pathName + "]");
            e.printStackTrace(System.err);
        }
    }
    
    public static enum Path {
        BLUE1(0), BLUE2(1), BLUE3(2), RED1(3), RED2(4), RED3(5), OFF(-1);

        public final int idx;

        private Path(final int idx) {
            this.idx = idx;
        }

        public Path toSide(final boolean isBlue) {
            switch(this) {
                case BLUE1:
                case RED1:
                    if(isBlue) {
                        return BLUE1;
                    }
                    return RED1;
                case BLUE2:
                case RED2:
                    if(isBlue) {
                        return BLUE2;
                    }
                    return RED2;
                case BLUE3:
                case RED3:
                    if(isBlue) {
                        return BLUE3;
                    }
                    return RED3;
                case OFF:
                default:
                    return OFF;
            }
        }
    }
}
