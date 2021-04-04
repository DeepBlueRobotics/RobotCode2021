package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GalacticSearchSetup extends CommandBase {
    
    private final Drivetrain drivetrain;
    private boolean isForward;

    public GalacticSearchSetup(Drivetrain drivetrain) {
        addRequirements(this.drivetrain = drivetrain);
    }

    @Override
    public void initialize() {
        isForward = false;
        Rotation2d rotation = new Rotation2d(drivetrain.getHeading());
        drivetrain.setOdometry(new SwerveDriveOdometry(drivetrain.getKinematics(), rotation, new Pose2d(new Translation2d(0, 0), rotation)));
    }

    @Override
    public void execute() {
        if(isForward) {
            drivetrain.drive(0, -1, 0);
        } else {
            drivetrain.drive(1, 0, 0);
        }
        if(drivetrain.getOdometry().getPoseMeters().getX() >= 1) {
            isForward = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getOdometry().getPoseMeters().getY()) >= 2;
    }

}
