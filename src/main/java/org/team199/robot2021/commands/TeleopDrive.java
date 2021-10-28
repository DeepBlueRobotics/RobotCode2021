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

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  //private static final double kSlowDriveSpeed = 0.6;
  //private static final double kSlowDriveRotation = 0.6;

  private final Drivetrain drivetrain;
  private Supplier<Double> fwd;
  private Supplier<Double> str;
  private Supplier<Double> rcw;
  double currentForward = 0;
  double currentStrafe = 0;
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
    double rawForward, rawStrafe, rotateClockwise, deltaT;
    deltaT = .05;
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    if (Math.abs(fwd.get()) <= Constants.OI.JOY_THRESH) rawForward = 0.0;
    else rawForward = Constants.DriveConstants.maxForward * fwd.get();
    if (Math.abs(str.get()) <= Constants.OI.JOY_THRESH) rawStrafe = 0.0;
    else rawStrafe = Constants.DriveConstants.maxStrafe * str.get();

    if (!SmartDashboard.getBoolean("Teleop Face Direction of Travel", false)) {
      rotateClockwise = rcw.get();
      if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
      else rotateClockwise = Constants.DriveConstants.maxRCW * rotateClockwise;
    } else {
      rotateClockwise = drivetrain.getHeadingDeg()/160;
    }
    //double currentForward = drivetrain.getSpeeds().vxMetersPerSecond;
    //double currentStrafe = -drivetrain.getSpeeds().vyMetersPerSecond;
    Vector2d targetAcceleration = new Vector2d((rawForward - currentForward)/deltaT, (rawStrafe - currentStrafe)/deltaT);
    double accelerationMagnitude = targetAcceleration.magnitude();
    if (accelerationMagnitude >= Constants.DriveConstants.autoMaxAccel) {
      targetAcceleration.x *= Constants.DriveConstants.autoMaxAccel/accelerationMagnitude;
      targetAcceleration.y *= Constants.DriveConstants.autoMaxAccel/accelerationMagnitude;
    }
    currentForward += targetAcceleration.x*deltaT;
    currentStrafe += targetAcceleration.y*deltaT;
    if (Math.abs(currentForward) <= Constants.OI.JOY_THRESH)
      currentForward = 0;
    if (Math.abs(currentStrafe) <= Constants.OI.JOY_THRESH)
      currentStrafe = 0;
    //SmartDashboard.putNumber("Forward (mps)", currentForward);
   // SmartDashboard.putNumber("Strafe (mps)", currentStrafe);
    drivetrain.drive(currentForward, currentStrafe, rotateClockwise);
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
