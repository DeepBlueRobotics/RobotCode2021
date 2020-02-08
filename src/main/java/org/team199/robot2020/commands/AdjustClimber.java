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
  private Climber climber;
  private Joystick controller;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getRawButton(Constants.OI.Controller.kAdjustClimberUpButton)) { // up
      climber.runLift(Climber.kLiftKeepSpeed + Climber.kLiftAdjustSpeed);
    } else if (controller.getRawButton(Constants.OI.Controller.kAdjustClimberDownButton)){ // down
      climber.runLift(Climber.kLiftKeepSpeed - Climber.kLiftAdjustSpeed);
    } else { // neutral
      climber.runLift(Climber.kLiftKeepSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runLift(Climber.kLiftKeepSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
