// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import java.util.ArrayList;

import org.team199.lib.RobotPath;
import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;

public class GalacticSearchSearch extends CommandBase {
  private final Drivetrain drivetrain;
  private final ArrayList<Translation2d> ballPositions;
  private final Limelight limelight;
  private Trajectory trajectory;
  /** Creates a new GalacticSearch. */
  public GalacticSearchSearch(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(this.drivetrain = drivetrain);
    this.limelight = limelight;
    ballPositions = new ArrayList<>();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d rotation = new Rotation2d(drivetrain.getHeading());
    drivetrain.setOdometry(new SwerveDriveOdometry(drivetrain.getKinematics(), rotation, new Pose2d(new Translation2d(0, 0), rotation)));
    ballPositions.clear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0, 1, 0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    if(tv == 1.0 && Math.abs(tx) <= 0.1) {
      boolean hasLogged = false;
      for(Translation2d pos: ballPositions) {
        if(Math.abs(pos.getY() - drivetrain.getOdometry().getPoseMeters().getTranslation().getY()) <= 0.5) {
          hasLogged = true;
          break;
        }
      }
      if(!hasLogged) {
        double[] dist = limelight.determineObjectDist(40, 3.5);
        ballPositions.add(new Translation2d(dist[0], drivetrain.getOdometry().getPoseMeters().getY() + (dist[1] / 39.37)));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      return;
    }
    drivetrain.drive(0, 0, 0);
    if(ballPositions.size() == 2) {
      Translation2d oldPos = null;
      for(Translation2d pos: ballPositions) {
        if(oldPos == null || oldPos.getX() > pos.getX()) {
          oldPos = pos;
        }
      }
      Translation2d newPos = new Translation2d(oldPos.getX(), oldPos.getY());
      newPos.plus(new Translation2d(3, 0));
      ballPositions.add(newPos);
    }
    Pose2d currentPosition = drivetrain.getOdometry().getPoseMeters();
    Translation2d currentTranslation = new Translation2d(currentPosition.getTranslation().getX(), currentPosition.getTranslation().getY());
    currentTranslation.plus(new Translation2d(7.5, 0));
    trajectory = TrajectoryGenerator.generateTrajectory(currentPosition, ballPositions, new Pose2d(currentTranslation, currentPosition.getRotation()), RobotPath.createConfig(false, drivetrain, Constants.DriveConstants.autoMaxSpeed));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballPositions.size() >= 3 || Math.abs(drivetrain.getOdometry().getPoseMeters().getY()) >= 4.5;
  }

  public Trajectory getTrajectory() {
    return trajectory;
  }
}
