package org.team199.robot2020.commands;

import org.team199.lib.Limelight;
import org.team199.lib.RobotPath;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootAndDrive extends SequentialCommandGroup {
    public AutoShootAndDrive(Drivetrain drivetrain, Intake intake, Feeder feeder, Limelight lime, RobotPath path) {
        // TODO: Make a shoot command group that ends when a certain number of balls have cleared the shooter.
        addCommands(
            new ShooterHorizontalAim(drivetrain, lime),
            new InstantCommand(() -> {
                intake.deploy();
                intake.intake();
            }, intake),
            path.getPathCommand(),
            new InstantCommand(() -> {
                intake.retract();
                intake.stop();
            }, intake)
        );
    }
}