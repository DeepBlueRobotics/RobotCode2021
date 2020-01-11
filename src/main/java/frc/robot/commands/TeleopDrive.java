/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Limelight;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
  Drivetrain dt;
  Joystick leftJoy, rightJoy;
  Limelight lime;
  private Limelight.Mode limelightMode = Limelight.Mode.STEER;
  private double minError = 0.05; // currently only used for steer and distance combo
  double prevSpeed = 0, prevLeft = 0, prevRight = 0;

  double outreachSpeed = 0.3;

  /**
   * Handles all the teleoperated driving functionality
   * 
   * @param dt the Drivetrain object to use, passing it in is useful for testing
   *           purposes
   */
  public TeleopDrive(final Drivetrain dt, final Limelight lime, final Joystick leftJoy, final Joystick rightJoy) {
    this.dt = dt;
    this.lime = lime;
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
    addRequirements(dt);
    
    SmartDashboard.putBoolean("Characterized Drive", false);
    
  }

    @Override
    public void execute() {
      if (SmartDashboard.getBoolean("Using Limelight", false)) {
        autoAlign();
      } else {
        if (SmartDashboard.getBoolean("Arcade Drive", true)) {
            arcadeDrive();
        } else {
            tankDrive();
        }
    }
    }

  private void arcadeDrive() {
    double speed = -leftJoy.getY();
    double rot = rightJoy.getX();

    // System.out.println("Speed: " + speed + ", Rotation: " + rot);
    SmartDashboard.putNumber("input speed", speed);
    SmartDashboard.putNumber("input rot", rot);

    if (SmartDashboard.getBoolean("Square Joysticks", true)) {
      speed = Math.copySign(speed * speed, speed);
      rot = Math.copySign(rot * rot, rot);
    }

    if (SmartDashboard.getBoolean("Slow Left", false)) {
      speed *= SmartDashboard.getNumber("Speed Slow Ratio", 0.5);
    }
    if (SmartDashboard.getBoolean("Slow Right", false)) {
      rot *= SmartDashboard.getNumber("Rotation Slow Ratio", 0.35);
    }

    if (SmartDashboard.getBoolean("Gradual Drive", true)) {
      final double dV = SmartDashboard.getNumber("Gradual Drive Max dV", 0.04);
      if (Math.abs(speed - prevSpeed) > dV) {
        speed = prevSpeed + dV * Math.signum(speed - prevSpeed);
      }
    }
    prevSpeed = speed;

    double left, right;

    // double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rot)),
    // speed);
    // copySign is returning incorrect signs in operation but not tests
    double maxInput = Math.max(Math.abs(speed), Math.abs(rot));

    if (speed >= 0.0) {
      // First quadrant, else second quadrant
      if (rot >= 0.0) {
        left = maxInput;
        right = speed - rot;
      } else {
        left = speed + rot;
        right = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      maxInput *= -1;
      if (rot >= 0.0) {
        left = speed + rot;
        right = maxInput;
      } else {
        left = maxInput;
        right = speed - rot;
      }
    }

    
      SmartDashboard.putBoolean("is in char drive", false);
      if (SmartDashboard.getBoolean("Outreach Mode", false)) {
        dt.drive(left * outreachSpeed, right * outreachSpeed);
      } else {
        dt.drive(left, right);
      }
    
  }

  private void tankDrive() {
    double left = -leftJoy.getY();
    double right = -rightJoy.getY();

    if (SmartDashboard.getBoolean("Square Joysticks", true)) {
      left = Math.copySign(left * left, left);
      right = Math.copySign(right * right, right);
    }

    if (SmartDashboard.getBoolean("Slow Left", false)) {
      left *= SmartDashboard.getNumber("Speed Slow Ratio", 0.5);
    }
    if (SmartDashboard.getBoolean("Slow Right", false)) {
      right *= SmartDashboard.getNumber("Speed Slow Ratio", 0.5);
    }

    if (SmartDashboard.getBoolean("Gradual Drive", true)) {
      final double dV = SmartDashboard.getNumber("Gradual Drive Max dV", 0.04);
      if (Math.abs(left - prevLeft) > dV) {
        left = prevLeft + dV * Math.signum(left - prevLeft);
      }
      if (Math.abs(right - prevRight) > dV) {
        right = prevRight + dV * Math.signum(right - prevRight);
      }
    }
    prevLeft = left;
    prevRight = right;

      if (SmartDashboard.getBoolean("Outreach Mode", false)) {
        dt.drive(left * outreachSpeed, right * outreachSpeed);
      } else {
        dt.drive(left, right);
      }
    
  }
  private void autoAlign() {
    double adjustment;
    if (limelightMode == Limelight.Mode.DIST) {
        adjustment = lime.distanceAssist();
        dt.drive(adjustment, adjustment);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
      else if (limelightMode == Limelight.Mode.STEER) {
        adjustment = lime.steeringAssist();
        final double[] charParams = dt.characterizedDrive(adjustment, -adjustment);
        dt.drive(charParams[0], -charParams[1]);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      } else {
        final double[] params = lime.autoTarget();
        dt.drive(params[0], params[1]);
        final double maxInput = Math.max(Math.abs(params[0]), Math.abs(params[1]));
        if (maxInput < minError)  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
  }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(final boolean interrupted) {
        dt.stop();
    }

}
