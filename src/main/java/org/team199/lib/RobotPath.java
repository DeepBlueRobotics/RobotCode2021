package org.team199.lib;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import org.team199.robot2020.subsystems.Drivetrain;

public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;
    private boolean isInverted;

    public RobotPath(String pathName, Drivetrain dt, boolean isInverted) throws IOException {
        trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("/home/lvuser/deploy/output/" + pathName + ".wpilib.json")));
        this.dt = dt;
        this.isInverted = isInverted;
    }

    public Command getPathCommand() {
        RamseteCommand ram = new RamseteCommand(trajectory, 
                                                () -> dt.getOdometry().getPoseMeters(), 
                                                new RamseteController(), 
                                                dt.getKinematics(),
                                                dt::charDriveDirect,
                                                dt);
        return new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.charDriveTank(0, 0), dt)); //TODO: Configure Ramsete Controller Values
    }

    public void loadOdometry() {
        if (!isInverted) {
            dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
        } else {
            // Invert the path in the future.
            dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
        }
    }
}