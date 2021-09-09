// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2021.subsystems.Drivetrain;

public class RotateAngle extends CommandBase {
  private Drivetrain dt;
  private double goalAngle;
  private double currentAngle;
  /** Creates a new RotateAngle. */
  public RotateAngle(Drivetrain dt, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.goalAngle = goalAngle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: determine speed angle rotates
    dt.drive(0, 0, 0.1);
    //TODO: use getHeading to find the angle we want the drivetrain to rotate. 
    //TODO: Change goalAngle to turnAngle, with goalAngle being the angle we want the drivetrian to rotate
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Determine margin of error
    return (Math.abs(goalAngle - currentAngle) < 5);
  }
}
