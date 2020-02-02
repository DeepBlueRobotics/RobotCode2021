
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
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.lib.Limelight;
import org.team199.robot2020.commands.TeleopDrive;
import org.team199.robot2020.commands.InitializeShoot;
import org.team199.robot2020.commands.ShooterTargetSpeed;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.PORT);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.PORT);
    private final Limelight lime;

    public RobotContainer() {
        configureButtonBindings();
        shooter.setDefaultCommand(new ShooterTargetSpeed(shooter));
        lime = new Limelight();
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, leftJoy, rightJoy, lime));
    }

    private void configureButtonBindings() {
        // characterize drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.CHARACTERIZED_DRIVE_BUTTON)
                .whenPressed(new InstantCommand(() -> SmartDashboard.putBoolean("Characterized Drive",
                        !SmartDashboard.getBoolean("Characterized Drive", false))));

        new JoystickButton(rightJoy, Constants.OI.RightJoy.SHOOT_BUTTON).whenPressed(new InitializeShoot(shooter));
        
        new JoystickButton(rightJoy, Constants.OI.RightJoy.LIMELIGHT_BUTTON)
            .whenPressed(new InstantCommand(() -> SmartDashboard.putBoolean("Using Limelight",
                !SmartDashboard.getBoolean("Using Limelight", false))));
    }

    public CommandBase getAutonomousCommand() {
        return null;
    }
}
