// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import frc.robot.lib.Limelight;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Feeder;
import org.team199.robot2021.subsystems.Intake;
import org.team199.robot2021.subsystems.Shooter;
import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallSearchAndPickup extends CommandBase {
  private Drivetrain drivetrain;
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  private Limelight lime;

  /** Creates a new BallSearchAndPickup. */
  public BallSearchAndPickup(Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, 
  Limelight lime) {
    this.lime = lime;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain, intake, feeder, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Search for Balls
    double[] distComponents = lime.determineObjectDist(Constants.DriveConstants.cameraHeight, 0.0, Constants.DriveConstants.cameraMountingAngleDeg);
    double xOffset = Constants.DriveConstants.xLimeDistIntake;
    double zOffset = Constants.DriveConstants.zLimeDistIntake;
    //(Make sure to subtract the horizontal dist. b/t limelight and center of intake from the left right dist)
    distComponents[0] = distComponents[0] - xOffset;
    distComponents[1] = distComponents[1] - zOffset;
    //Find angle and distance from balls using trigonometry and getObjectDistance method in Limelight class (0 is forward, 1 is strafe)
    double angle = Math.atan2(distComponents[0], distComponents[1]);
    double distance = Math.hypot(distComponents[0], distComponents[1]);
    //Rotate Robot along angle till it's facing ball
      //Create new command to do this where isfinished checks if the goal angle is the same as the current angle
    //Run the intake
    //Drive robot forward the distance plus a little bit
    //Congrats you're done
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

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
