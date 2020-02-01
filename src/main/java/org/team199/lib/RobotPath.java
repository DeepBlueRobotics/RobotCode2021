package org.team199.lib;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.team199.robot2020.subsystems.Drivetrain;

public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;
    private boolean isInit;

    public RobotPath(String pathName) throws IOException {
        trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + pathName + ".wpilib.json"));
        isInit = false;
    }

    public void init(Drivetrain dt) {
        if (isInit) {
            return;
        }
        this.dt = dt;
        isInit = true;
    }

    public Command getPathCommand() {
        if(!isInit) {
            return new RunCommand(() -> {});
        }
        if(dt.getOdometry() == null) {
            Trajectory.State state = trajectory.getStates().get(0);
            dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), state.poseMeters));
        }
        return new RamseteCommand(trajectory, () -> dt.getOdometry().getPoseMeters(),
        new RamseteController(2, 0.7), dt.getKinematics(), dt::charDriveTank, dt)
        .andThen(() -> dt.charDriveTank(0, 0), dt); //TODO: Configure Ramsete Controller Values
    }

}