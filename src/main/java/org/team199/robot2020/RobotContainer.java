/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.IOException;

import org.team199.lib.RobotPath;
import org.team199.robot2020.commands.Regurgitate;
import org.team199.robot2020.commands.TeleopDrive;
import org.team199.robot2020.commands.AdjustClimber;
import org.team199.robot2020.commands.DeployClimber;
import org.team199.robot2020.commands.RaiseRobot;
import org.team199.robot2020.subsystems.Drivetrain;
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
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.kPort);
    private final Joystick controller = new Joystick(Constants.OI.Controller.kPort);
    private final Climber climber = new Climber();


    private RobotPath path;

    public RobotContainer() {
        configureButtonBindings();
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, leftJoy, rightJoy));
        feeder.setDefaultCommand(new RunCommand(() -> {
            if (feeder.isBallEntering()) 
                feeder.runForward();
            else 
                feeder.stop();
        }));

        try {
            path = new RobotPath("Test2");
        } catch (IOException e) {
            e.printStackTrace();
        }
        path.init(drivetrain);
    }

    private void configureButtonBindings() {
        // Arcade/Tank drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.kToggleDriveModeButton).whenPressed(new InstantCommand(
                () -> SmartDashboard.putBoolean("Arcade Drive", !SmartDashboard.getBoolean("Arcade Drive", false))));

        // characterize drive button
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

        // climb button
        new JoystickButton(controller, Constants.OI.Controller.kRaiseRobotButton).whenPressed(new RaiseRobot(climber));

    }

    public Command getAutonomousCommand() {
        return path.getPathCommand();
    }
}