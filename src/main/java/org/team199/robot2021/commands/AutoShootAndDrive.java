package org.team199.robot2021.commands;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.lib.Limelight;
import frc.robot.lib.LinearInterpolation;
import org.team199.lib.RobotPath;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Feeder;
import org.team199.robot2021.subsystems.Intake;
import org.team199.robot2021.subsystems.Shooter;
import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootAndDrive extends SequentialCommandGroup {
    public AutoShootAndDrive(Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, Turret turret,
                             Limelight lime, RobotPath path, LinearInterpolation linearInterpol, Translation2d target) {
        addRequirements(drivetrain, intake, feeder, shooter, turret);

        TimeOfFlight shooterDistanceSensor = feeder.getShooterDistanceSensor();
        ShooterHorizontalAim aim = new ShooterHorizontalAim(turret, lime);
        AutoShoot shoot = new AutoShoot(feeder, shooter, shooterDistanceSensor, 3);

        /*
        addCommands(
            aim,
            new InstantCommand(() -> { 
                SmartDashboard.putNumber("Shooter.kTargetSpeed", linearInterpol.calculate(drivetrain.getOdometry().getPoseMeters().getTranslation().getDistance(target))); 
            }),
            shoot,
            new ShooterHorizontalAim(turret, lime),
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
            new InstantCommand(() -> { 
                SmartDashboard.putNumber("Shooter.kTargetSpeed", linearInterpol.calculate(drivetrain.getOdometry().getPoseMeters().getTranslation().getDistance(target))); 
            }),
            shoot
        );
        */
    }
}