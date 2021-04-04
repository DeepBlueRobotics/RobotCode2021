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
    //resets odometry
    public void initialize() {
        isForward = false;
        Rotation2d rotation = new Rotation2d(drivetrain.getHeading());
        drivetrain.setOdometry(new SwerveDriveOdometry(drivetrain.getKinematics(), rotation, new Pose2d(new Translation2d(0, 0), rotation)));
    }

    @Override
    /*Moves the robot forward 1 meter and the moves upward */
    /* This is done because there are cones on the start zone boundary that would obstruct balls, so the robot moves forward out of the way of the cones */
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
    /*finishes command when the robot has moved to the top of the field*/
    public boolean isFinished() {
        return Math.abs(drivetrain.getOdometry().getPoseMeters().getY()) >= 2;
    }

}
