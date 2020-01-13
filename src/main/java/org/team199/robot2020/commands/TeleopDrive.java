/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  Drivetrain drivetrain;
  DoubleSupplier speed, rotation;
  BooleanSupplier leftSlow, rightSlow;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier rotation, BooleanSupplier leftSlow, BooleanSupplier rightSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain);
    this.speed = speed;
    this.rotation = rotation;
    this.leftSlow = leftSlow;
    this.rightSlow = rightSlow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s = speed.getAsDouble();
    double r = rotation.getAsDouble();

    if (leftSlow.getAsBoolean()) {
      s *= Constants.SLOW_DRIVE;
    }

    if (rightSlow.getAsBoolean()) {
      r *= Constants.SLOW_DRIVE;
    }

    drivetrain.arcadeDrive(s, r);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
