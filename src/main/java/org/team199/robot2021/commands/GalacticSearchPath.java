// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import org.team199.lib.RobotPath;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GalacticSearchPath extends SequentialCommandGroup {

  private final Drivetrain drivetrain;
  private final Intake intake;
  private GalacticSearchSearch search;
  private Command command;

  /** Creates a new GalacticSearch. */
  public GalacticSearchPath(Drivetrain drivetrain, Intake intake, GalacticSearchSearch search) {
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.intake = intake);
    this.search = search;
  }

  @Override
  public void initialize() {
    command = new RobotPath(search.getTrajectory(), drivetrain, intake, false).getPathCommand(true);
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

}
