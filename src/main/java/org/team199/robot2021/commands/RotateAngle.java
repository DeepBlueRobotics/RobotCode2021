// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2021.subsystems.Drivetrain;

public class RotateAngle extends CommandBase {
  private Drivetrain drivetrain;
  private double rotateAngle;
  private double currentAngle;
  private double goalAngle;
  /** Creates a new RotateAngle. */
  public RotateAngle(Drivetrain drivetrain, double rotateAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.rotateAngle = rotateAngle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: determine speed angle rotates
    //Rotates based on if rotateAngle is positive or negative
    //TODO: define goalAngle as the final angle based on odometry values
    //TODO: ensure getHeading returns an angle
    goalAngle = drivetrain.getHeadingDeg() *Math.PI/180 + rotateAngle;
    if (Math.abs(rotateAngle) != rotateAngle) {
      drivetrain.drive(0, 0, -0.1);
    } else {
      drivetrain.drive(0, 0, 0.1);
    }
    //TODO: use getHeading to find the angle we want the drivetrain to rotate. 
    //TODO: Change goalAngle to turnAngle, with goalAngle being the angle we want the drivetrian to rotate
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = drivetrain.getHeadingDeg()*Math.PI/180;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0 , 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Determine margin of error
    return (Math.abs(goalAngle - currentAngle) < 5);
  }
}
