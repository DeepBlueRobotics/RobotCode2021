/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import java.util.function.Supplier;

import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  //private static final double kSlowDriveSpeed = 0.6;
  //private static final double kSlowDriveRotation = 0.6;

  private final Drivetrain drivetrain;
  private Supplier<Double> fwd;
  private Supplier<Double> str;
  private Supplier<Double> rcw;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, Supplier<Double> fwd, Supplier<Double> str, Supplier<Double> rcw) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward, strafe, rotateClockwise;
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    if (Math.abs(fwd.get()) <= Constants.OI.JOY_THRESH) forward = 0.0;
    else forward = Constants.DriveConstants.maxForward * fwd.get();
    if (Math.abs(str.get()) <= Constants.OI.JOY_THRESH) strafe = 0.0;
    else strafe = Constants.DriveConstants.maxStrafe * str.get();

    if (!SmartDashboard.getBoolean("Teleop Face Direction of Travel", false)) {
      if (Math.abs(rcw.get()) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
      else rotateClockwise = Constants.DriveConstants.maxRCW * rcw.get();
    } else {
      rotateClockwise = drivetrain.getHeading()/90;
    }
    
    drivetrain.drive(forward, strafe, rotateClockwise);
  }

  /*
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
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
