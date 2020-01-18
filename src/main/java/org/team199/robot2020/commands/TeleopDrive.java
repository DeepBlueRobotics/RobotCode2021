/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.lib.Limelight;
import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private Drivetrain drivetrain;
  private Joystick leftJoy, rightJoy;
  Limelight lime;
  private Limelight.Mode limelightMode = Limelight.Mode.STEER;
  private double minError = 0.05; // currently only used for steer and distance combo

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, Joystick leftJoy, Joystick rightJoy, Limelight lime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain);
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
    this.lime = lime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("Using Limelight", false)) {
      autoAlign();
    } else {
      double speed = -leftJoy.getY();
      double rotation = rightJoy.getX();
      if (leftJoy.getRawButton(Constants.OI.LeftJoy.SLOW_DRIVE_BUTTON)) {
        speed *= Constants.SLOW_DRIVE_SPEED;
      }

      if (rightJoy.getRawButton(Constants.OI.RightJoy.SLOW_DRIVE_BUTTON)) {
        rotation *= Constants.SLOW_DRIVE_ROTATION;
      }
      drivetrain.arcadeDrive(speed, rotation);
    }
  }

  private void autoAlign() {
    double adjustment;
    if (limelightMode == Limelight.Mode.DIST) {
        adjustment = lime.distanceAssist();
        drivetrain.tankDrive(adjustment, adjustment);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
      else if (limelightMode == Limelight.Mode.STEER) {
        adjustment = lime.steeringAssist();
        final double[] charParams = drivetrain.characterizedDrive(adjustment, -adjustment);
        drivetrain.tankDrive(charParams[0], -charParams[1]);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      } else {
        final double[] params = lime.autoTarget();
        drivetrain.tankDrive(params[0], params[1]);
        final double maxInput = Math.max(Math.abs(params[0]), Math.abs(params[1]));
        if (maxInput < minError)  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
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
