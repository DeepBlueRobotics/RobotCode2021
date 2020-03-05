package org.team199.robot2020.commands;

import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.Limelight;
import org.team199.lib.RobotPath;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;
import org.team199.robot2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootAndDrive extends SequentialCommandGroup {
    public AutoShootAndDrive(Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, 
                             Limelight lime, RobotPath path, Translation2d target) {
        addRequirements(drivetrain, intake, feeder, shooter);

        TimeOfFlight shooterDistanceSensor = feeder.getShooterDistanceSensor();
        ShooterHorizontalAim aim = new ShooterHorizontalAim(drivetrain, lime);
        AutoShoot shoot = new AutoShoot(feeder, intake, shooter, shooterDistanceSensor, 3);

        addCommands(
            aim,
            shoot,
            new ShooterHorizontalAim(drivetrain, lime),
            new InstantCommand(() -> {
                intake.intake();
                intake.doTheFlop();
            }, intake),
            path.getPathCommand(),
            new InstantCommand(() -> {
                intake.retract();
                intake.stop();
            }, intake),
            aim,
            shoot
        );
    }
}