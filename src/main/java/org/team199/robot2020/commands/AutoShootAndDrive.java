package org.team199.robot2020.commands;

import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.Limelight;
import org.team199.lib.RobotPath;
import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;
import org.team199.robot2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootAndDrive extends SequentialCommandGroup {
    public AutoShootAndDrive(Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, 
                             Limelight lime, RobotPath path, Translation2d target, PowerDistributionPanel pdp, int feederPDPPort) {

        TimeOfFlight shooterDistanceSensor = feeder.getShooterDistanceSensor();
        ShooterHorizontalAim aim = new ShooterHorizontalAim(drivetrain, lime);
        AutoShoot shoot = new AutoShoot(feeder, intake, shooter, shooterDistanceSensor, 3);
        AutoBallPickup pickup = new AutoBallPickup(feeder, intake, drivetrain, pdp, feederPDPPort);

        addCommands(
            aim,
            shoot,
            new ParallelCommandGroup(
                path.getPathCommand(),
                pickup
            ),
            aim,
            shoot
        );
    }
}