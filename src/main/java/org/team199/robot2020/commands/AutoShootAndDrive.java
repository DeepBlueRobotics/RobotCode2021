package org.team199.robot2020.commands;

import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.Limelight;
import org.team199.lib.RobotPath;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;
import org.team199.robot2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootAndDrive extends SequentialCommandGroup {
    public AutoShootAndDrive(Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, Limelight lime, RobotPath path) {
        addRequirements(drivetrain, intake, feeder, shooter);

        TimeOfFlight shooterDistanceSensor = feeder.getShooterDistanceSensor();
        SequentialCommandGroup initShoot = new SequentialCommandGroup(
            //new ShooterHorizontalAim(drivetrain, lime), 
            new AutoShoot(feeder, shooter, shooterDistanceSensor, 3, 5500)
        );
        SequentialCommandGroup finalShoot = new SequentialCommandGroup(
            //new ShooterHorizontalAim(drivetrain, lime), 
            new AutoShoot(feeder, shooter, shooterDistanceSensor, 5, 4200)
        );

        addCommands(
            initShoot,
            new InstantCommand(() -> {
                intake.intake();
                intake.doTheFlop();
            }),
            path.getPathCommand(),
            new InstantCommand(() -> {
                intake.retract();
                intake.stop();
            }),
            finalShoot
        );
    }
}