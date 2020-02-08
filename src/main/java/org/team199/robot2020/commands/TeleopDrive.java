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
      if (Math.abs(speed) < 0.001) { speed = 0.0; }
      if (Math.abs(rotation) < 0.001) { rotation = 0.0; }

      if (leftJoy.getRawButton(Constants.OI.LeftJoy.SLOW_DRIVE_BUTTON)) {
        speed *= Constants.SLOW_DRIVE_SPEED;
      }

      if (rightJoy.getRawButton(Constants.OI.RightJoy.SLOW_DRIVE_BUTTON)) {
        rotation *= Constants.SLOW_DRIVE_ROTATION;
      }

      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        drivetrain.charDriveArcade(speed, rotation);
      } else {
        drivetrain.arcadeDrive(speed, rotation);
      }
    } else {
      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        drivetrain.charDriveTank(-leftJoy.getY(), -rightJoy.getY());
      } else {
        drivetrain.tankDrive(-leftJoy.getY(), -rightJoy.getY(), true);
      }
    }
    if (drivetrain.getKinematics() == null) { System.out.println("1"); }
    /*if (drivetrain.getOdometry() == null) { System.out.println("1"); }
    else if (drivetrain.getOdometry().getPoseMeters() == null) { System.out.println("2"); }
    else if (drivetrain.getOdometry().getPoseMeters().getTranslation() == null) { System.out.println("3"); }
    SmartDashboard.putNumber("PoseMeters", drivetrain.getOdometry().getPoseMeters().getTranslation().getX());*/

    SmartDashboard.putNumber("Left Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.RIGHT));
    SmartDashboard.putNumber("Left Encoder Distance", drivetrain.getEncPos(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Distance", drivetrain.getEncPos(Drivetrain.Side.RIGHT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
