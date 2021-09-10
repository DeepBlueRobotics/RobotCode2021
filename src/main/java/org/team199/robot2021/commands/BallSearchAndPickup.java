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
import org.team199.robot2021.commands.RotateAngle;
import org.team199.robot2021.commands.DriveDistance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallSearchAndPickup extends SequentialCommandGroup {
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
    RotateAngle check = new RotateAngle(drivetrain, Math.PI/5);

    for (int i = 0; i < 10; i++) {
      addCommands(check);
      if (SmartDashboard.getNumber("Found Vision Target", 0) != 0) {
        break;
      }
    }

    if (SmartDashboard.getNumber("Found Vision Target", 0) != 0) {
      // Use addRequirements() here to declare subsystem dependencies.
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
      RotateAngle rotate = new RotateAngle(drivetrain, angle);
      DriveDistance drive = new DriveDistance(drivetrain, distance);
      //Rotate Robot along angle till it's facing ball
        //Create new command to do this where isfinished checks if the goal angle is the same as the current angle
      //Run the intak
      addCommands(rotate, new InstantCommand(() -> intake.deploy()), new InstantCommand(() -> intake.intake()), drive, new InstantCommand(() -> intake.stop()), new InstantCommand(() -> intake.retract()));
    }
  }

  // Called when the command is initially scheduled.
  
}
