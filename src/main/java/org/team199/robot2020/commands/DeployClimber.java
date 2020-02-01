/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployClimber extends CommandBase {
  private Climber climber;

  /**
   * Deploys climber up to switch
   */
  public DeployClimber(Climber climber) {
    addRequirements(this.climber = climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runLift(Constants.Climber.LIFT_UP_SPEED);
    climber.runWinch(Constants.Climber.WINCH_UP_SPEED);
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
    climber.runLift(Constants.Climber.LIFT_KEEP_SPEED);
    climber.runWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getLiftHeight() >= Constants.Climber.LIFT_HEIGHT;
  }
}
