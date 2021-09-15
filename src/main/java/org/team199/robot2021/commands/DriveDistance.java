// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private Drivetrain drivetrain;
  private double distance;
  private Translation2d initDist;
  private double hasDriven;
  /** Creates a new DriveDistance. */
  public DriveDistance(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(1, 0, 0);
    //Sets initial Distance
    initDist = drivetrain.getOdometry().getPoseMeters().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Keep Going forward (Set speed of wheels or whatever that is)
    drivetrain.drive(1,0,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    hasDriven = drivetrain.getOdometry().getPoseMeters().getTranslation().getDistance(initDist);
    return (hasDriven >= distance);
  }
}
