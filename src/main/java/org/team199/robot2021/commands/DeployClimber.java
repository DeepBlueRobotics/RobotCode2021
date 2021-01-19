/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployClimber extends CommandBase {
  private final Climber climber;

  /**
   * Deploys climber up to switch
   */
  public DeployClimber(Climber climber) {
    addRequirements(this.climber = climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runLift(Climber.kLiftDeploySpeed);
    climber.runWinch(Climber.kWinchDeploySpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (climber.getWinchHeight() >= 0) {
        climber.runWinch(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runLift(Climber.kLiftKeepSpeed);
    climber.runWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getLiftHeight() >= Climber.kLiftHeight;
  }
}
