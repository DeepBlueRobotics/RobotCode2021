/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AdjustClimber extends CommandBase {
  private final Climber climber;
  private final Joystick controller;

  /**
   * Manually adjusting the climber hook position
   */
  public AdjustClimber(Climber climber, Joystick manipulator) {
    addRequirements(this.climber = climber);
    this.controller = manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pulls lift and winch in, arm will go all the way down, winch will stop once hook reaches bar
    climber.runLift(climber.kLiftLowerSpeed);
    climber.runLift(climber.kWinchRetractSpeedFirst);
    System.out.println("--ADJUST CLIMBER--");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Stops the winch when the hook reaches the bar
    if (climber.getWinchHeight() <= climber.kWinchMidHeight) {
      climber.runWinch(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop lift and winch when arm returns to robot
    climber.runLift(0);
    climber.runWinch(0);
    System.out.println("--ENDED--");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checks if arm has returned to robot
    return climber.getLiftHeight() <= Climber.kLiftLowerHeight;
    //TODO: stop when it reaches bottom. (call end)
  }
}
