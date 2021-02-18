/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseRobot extends CommandBase {
  private final Climber climber;

  /**
   * Pulls the winch to raise the robot up to the switch
   * Old needs to be fixed and updated for current
   */
  public RaiseRobot(Climber climber) {
    addRequirements(this.climber = climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runWinch(Climber.kWinchRetractSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getWinchHeight() >= Climber.kWinchEndHeight;
  }
}