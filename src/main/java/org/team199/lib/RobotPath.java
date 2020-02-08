package org.team199.lib;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
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

    public RobotPath(String pathName, Drivetrain dt) throws IOException {
        trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("/home/lvuser/deploy/output/" + pathName + ".wpilib.json")));
        this.dt = dt;
    }

    public Command getPathCommand() {
        return new InstantCommand(this::loadOdometry).andThen(new RamseteCommand(trajectory,
        () -> dt.getOdometry().getPoseMeters(), new RamseteController(), dt.getKinematics(),
        dt::charDriveDirect, dt), new InstantCommand(() -> dt.charDriveTank(0, 0), dt)); //TODO: Configure Ramsete Controller Values
    }

    public void loadOdometry() {
        dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
    }
}