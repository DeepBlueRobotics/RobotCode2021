
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

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
    private final DigitalInput autoSwitch1 = new DigitalInput(Constants.Drivetrain.AUTO_PATH_SWITCH_1_PORT);
    private final DigitalInput autoSwitch2 = new DigitalInput(Constants.Drivetrain.AUTO_PATH_SWITCH_2_PORT);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.kPort);
    private final Joystick controller = new Joystick(Constants.OI.Controller.kPort);
    private final Climber climber = new Climber();
    private final RobotPath[] paths;

    private final Limelight lime = new Limelight();

    public RobotContainer() {
        configureButtonBindings();
        shooter.setDefaultCommand(new RunCommand(()-> shooter.setSpeed(shooter.getTargetSpeed()), shooter));
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, leftJoy, rightJoy, lime));
        
        feeder.setDefaultCommand(new RunCommand(() -> {
            if (feeder.isCellEntering() && !feeder.isCellAtShooter()) 
                feeder.runForward();
            else 
                feeder.stop();
        }, feeder));

        paths = new RobotPath[6];
        loadPath(Path.BLUE1, "Blue1", true);
        loadPath(Path.BLUE2, "Blue2", true);
        loadPath(Path.BLUE3, "Blue3", true);
        loadPath(Path.RED1, "Red1", true);
        loadPath(Path.RED2, "Red2", true);
        loadPath(Path.RED3, "Red3", true);
    }

    private void configureButtonBindings() {
        // Arcade/Tank drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kToggleDriveModeButton).whenPressed(new InstantCommand(
                () -> SmartDashboard.putBoolean("Arcade Drive", !SmartDashboard.getBoolean("Arcade Drive", false))));

        // characterize drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kCharacterizedDriveButton)
                .whenPressed(new InstantCommand(() -> SmartDashboard.putBoolean("Characterized Drive",
                        !SmartDashboard.getBoolean("Characterized Drive", false))));
        
        // Toggle Characterize Drive                
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kCharacterizedDriveButton).whenPressed(new InstantCommand(
                () -> SmartDashboard.putBoolean("Characterized Drive", !SmartDashboard.getBoolean("Characterized Drive", false))));

        // Intake toggle button
        new JoystickButton(controller, Constants.OI.Controller.kIntakeButton).whenPressed(new InstantCommand(() -> {
            if (intake.isDeployed()) {
                intake.retract();
                intake.stop();
            } else {
                intake.deploy();
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

        // Align the robot and then shoots
        new JoystickButton(rightJoy, Constants.OI.RightJoy.kAlignAndShootButton).whileHeld(new SequentialCommandGroup(new ShooterHorizontalAim(drivetrain, lime), new Shoot(feeder)));

        // climb button
        new JoystickButton(controller, Constants.OI.Controller.kRaiseRobotButton).whenPressed(new RaiseRobot(climber));

    }

    public Command getAutonomousCommand() {
        try {
            final RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            }
            return new AutoShootAndDrive(intake, path);
        } catch(final Exception e) {
            return new InstantCommand();
        }
    }

    /**
     * DIO Port 0 = Switch 1
     * DIO Port 1 = Switch 2
     * on = jumper in
     * off= jumper out
     * connect jumpers between 5V and S NOT 5V and GND
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
        if(autoSwitch1.get()) {
            if(autoSwitch2.get()) {
                outPath = Path.BLUE3;
            } else {
                outPath = Path.BLUE1;
            }
        } else if(autoSwitch2.get()) {
            outPath = Path.BLUE2;
        } else {
            outPath = Path.OFF;
        }
        return outPath.toSide(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);
    }

    private void loadPath(final Path path, final String pathName, final boolean isInverted) {
        try {
            paths[path.idx] = new RobotPath(pathName, drivetrain, isInverted);
        } catch(final Exception e) {
            System.err.println("Error Occured Loading Path: [" + path.name() + "," + pathName + "]");
            e.printStackTrace(System.err);
        }
    }public static enum Path {
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
