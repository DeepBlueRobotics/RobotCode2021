
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
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

import org.team199.lib.Limelight;
import org.team199.lib.LinearInterpolation;
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
    private final DigitalInput autoSwitch1 = new DigitalInput(Constants.Ports.kAutoPathSwitch1Port);
    private final DigitalInput autoSwitch2 = new DigitalInput(Constants.Ports.kAutoPathSwitch2Port);
    private final DigitalInput autoSwitch3 = new DigitalInput(Constants.Ports.kAutoPathSwitch3Port);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Limelight lime = new Limelight();
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.kPort);
    private final Joystick controller = new Joystick(Constants.OI.Controller.kPort);
    private final Climber climber = new Climber();
    private final RobotPath[] paths;
    private final LinearInterpolation linearInterpol = new LinearInterpolation("ShooterData.csv");
    private final boolean isBlue = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue;
    private final Shooter shooter = new Shooter(drivetrain, lime, linearInterpol, isBlue ? Constants.FieldPositions.BLUE_PORT : Constants.FieldPositions.RED_PORT);

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
            } else {
                feeder.stop();
            }
        }, feeder));

        // Old Intake default command:
        /*
        intake.setDefaultCommand(new RunCommand(() -> {
            if(intake.isDeployed()) {
                if(feeder.has5Intake()) {
                    intake.stop();
                } else if(feeder.isIntakeCellEntering()) {
                    intake.slow();
                } else {
                    intake.intake();
                }
            } else {
                intake.stop();
            }
        }, intake));
        */
        intake.setDefaultCommand(new RunCommand(() -> {
            boolean encoderReset = false;
            double targetEncoderDist = 100.0;   // TODO: Figure out the correct value.
            if(intake.isDeployed()) {
                if(feeder.isIntakeCellEntering()) {
                    if (!feeder.isCellAtShooter()) {
                        intake.slow();
                    } else {
                        if (!encoderReset) {
                            intake.resetEncoder();
                            encoderReset = true;
                        }
                        if (intake.getEncoderDistance() <= targetEncoderDist) {
                            intake.slow();
                        } else {
                            encoderReset = false;
                            intake.stop();
                        }
                    }
                } else {
                    intake.intake();
                }
            } else {
                intake.stop();
            }
        }, intake));

        paths = new RobotPath[7];
        loadDPath(Path.PATH1, "AutoLeft", false, Constants.FieldPositions.RED_LEFT.pos, Constants.FieldPositions.BLUE_LEFT.pos);
        loadDPath(Path.PATH2, "OneBall", false, Constants.FieldPositions.RED_CENTER.pos, Constants.FieldPositions.BLUE_CENTER.pos);
        loadDPath(Path.PATH3, "AutoRight", false, Constants.FieldPositions.RED_RIGHT.pos, Constants.FieldPositions.BLUE_RIGHT.pos);
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

    private void configureButtonBindingsRightJoy() {new JoystickButton(rightJoy, 3).whenPressed(new InstantCommand(drivetrain::toggleBreakMode, drivetrain));
        // Align the robot and then shoots
        new JoystickButton(rightJoy, Constants.OI.RightJoy.kAlignAndShootButton).whileHeld(new SequentialCommandGroup(new ShooterHorizontalAim(drivetrain, lime), new Shoot(feeder, intake)));
        new JoystickButton(rightJoy, Constants.OI.RightJoy.kShootButton).whileHeld(new Shoot(feeder, intake));
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

    final Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        try {
            final RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            }
            return new AutoShootAndDrive(drivetrain, intake, feeder, 
                                         shooter, lime, path, 
                                         (isBlue ? Constants.FieldPositions.BLUE_PORT.pos : Constants.FieldPositions.RED_PORT.pos));
        } catch(final Exception e) {
            return new InstantCommand();
        }
    }

    
    /**
     * DIO Port 0 = Switch 1
     * DIO Port 1 = Switch 2
     * DIO Port 2 = Switch 3
     * on = jumper in
     * off= jumper out
     * Red/Blue determined by DS
     * Switch states
     * 1    2
     * off off off = off
     * on off off = 1
     * off on off = 2
     * on on off= 3
     * off off on = off
     * on off on = 1
     * off on on = 2
     * on on on = 3
     */
    public Path getPath() {
        // get() returns true if the circuit is open.
        int b1 = autoSwitch1.get() ? 0 : 1;
        int b2 = autoSwitch2.get() ? 0 : 1;
        int b3 = autoSwitch3.get() ? 0 : 1;
        return Path.fromIdx((b1 | b2 << 1 | b3 << 2)-1);
    }

    private void loadDPath(Path path, String pathName, boolean isInverted, Translation2d redInitPos, Translation2d blueInitPos) {
        loadPath(path, pathName, isInverted, DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue ? blueInitPos : redInitPos);
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
        PATH1(0), PATH2(1), PATH3(2), PATH4(3), PATH5(4), PATH6(5), PATH7(6), OFF(-1);

        public final int idx;

        private Path(final int idx) {
            this.idx = idx;
        }

        public static final Path fromIdx(int idx) {
            switch(idx) {
                case 0:
                return PATH1;
                case 1:
                return PATH2;
                case 2:
                return PATH3;
                case 3:
                return PATH4;
                case 4:
                return PATH5;
                case 5:
                return PATH6;
                case 6:
                return PATH7;
                case -1:
                default:
                return OFF;
            }
        }
    }
}
