/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AttachHook extends CommandBase {
  private final Climber climber;
  private final Joystick controller;
  private boolean hookAttached;
  /**
   * Manually adjusting the climber hook position
   */
  public AttachHook(Climber climber, Joystick manipulator) {
    addRequirements(this.climber = climber);
    this.controller = manipulator;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setWinchIdleBrake();
    //pulls lift and winch in, arm will go all the way down, winch will stop once hook reaches bar
    hookAttached = climber.isHookAttached();
    climber.runLift(climber.kLiftRetractSpeed);
    System.out.println("--ATTACH HOOK--");
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
    return !hookAttached || (climber.getLiftHeight() <= climber.kArmRetryDistance);
  }
}
