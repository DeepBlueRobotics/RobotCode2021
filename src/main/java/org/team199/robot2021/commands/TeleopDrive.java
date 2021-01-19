/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.lib.Limelight;
import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private static final double kSlowDriveSpeed = 0.6;
  private static final double kSlowDriveRotation = 0.6;

  private final Drivetrain drivetrain;
  private final Joystick leftJoy, rightJoy;
  private final Limelight lime;
  private Limelight.Mode limelightMode = Limelight.Mode.STEER;
  private final double minError = 0.05; // currently only used for steer and distance combo

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
    boolean slowLeft = leftJoy.getRawButton(Constants.OI.LeftJoy.kSlowDriveButton);
    boolean slowRight = rightJoy.getRawButton(Constants.OI.RightJoy.kSlowDriveButton);

    if (SmartDashboard.getBoolean("Arcade Drive", true)) {
      double speed = -leftJoy.getY();
      double rotation = rightJoy.getX();
      if (Math.abs(speed) < 0.001) { speed = 0.0; }
      if (Math.abs(rotation) < 0.001) { rotation = 0.0; }

      if (slowLeft) speed *= kSlowDriveSpeed;
      if (slowRight) rotation *= kSlowDriveRotation;

      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        drivetrain.charDriveArcade(speed, rotation);
      } else {
        drivetrain.arcadeDrive(speed, rotation);
      }
    } else {
      double left = -leftJoy.getY();
      double right = -rightJoy.getX();

      if (slowLeft) left *= kSlowDriveSpeed;
      if (slowRight) right *= kSlowDriveRotation;

      if (SmartDashboard.getBoolean("Characterized Drive", false)) {
        drivetrain.charDriveTank(left, right);
      } else {
        drivetrain.tankDrive(left, right, true);
      }
    }
  }

  private void autoAlign() {
    double adjustment;
    if (limelightMode == Limelight.Mode.DIST) {
        adjustment = lime.distanceAssist();
        drivetrain.tankDrive(adjustment, adjustment, false);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
      else if (limelightMode == Limelight.Mode.STEER) {
        adjustment = lime.steeringAssist();
        //final double[] charParams = drivetrain.characterizedDrive(adjustment, -adjustment);
        drivetrain.tankDrive(adjustment, -adjustment, false);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      } else {
        final double[] params = lime.autoTarget();
        drivetrain.tankDrive(params[0], params[1], false);
        final double maxInput = Math.max(Math.abs(params[0]), Math.abs(params[1]));
        if (maxInput < minError)  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
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
