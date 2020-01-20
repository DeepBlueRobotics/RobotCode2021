/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private Drivetrain drivetrain;
  private Joystick leftJoy, rightJoy;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, Joystick leftJoy, Joystick rightJoy) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain);
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("Arcade Drive", true)) {
      double speed = -leftJoy.getY();
      double rotation = rightJoy.getX();

      if (leftJoy.getRawButton(Constants.OI.LeftJoy.SLOW_DRIVE_BUTTON)) {
        speed *= Constants.SLOW_DRIVE_SPEED;
      }

      if (rightJoy.getRawButton(Constants.OI.RightJoy.SLOW_DRIVE_BUTTON)) {
        rotation *= Constants.SLOW_DRIVE_ROTATION;
      }

      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);
    
        double left, right;
        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
    
        if (speed >= 0.0) {
          // First quadrant, else second quadrant
          if (rotation >= 0.0) {
            left = maxInput;
            right = speed - rotation;
          } else {
            left = speed + rotation;
            right = maxInput;
          }
        } else {
          // Third quadrant, else fourth quadrant
          if (rotation >= 0.0) {
            left = speed + rotation;
            right = maxInput;
          } else {
            left = maxInput;
            right = speed - rotation;
          }
        }
        drivetrain.characterizedDrive(left, right);
      } else {
        drivetrain.arcadeDrive(speed, rotation);
      }
    } else {
      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        drivetrain.characterizedDrive(-leftJoy.getY(), -rightJoy.getY());
      } else {
        drivetrain.tankDrive(-leftJoy.getY(), -rightJoy.getY());
      }
    }

    SmartDashboard.putNumber("Left Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Left Encoder Distance", drivetrain.getEncRate(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Distance", drivetrain.getEncRate(Drivetrain.Side.LEFT));
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
