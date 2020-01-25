/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Climber;

public class RaiseRobot extends CommandBase {
  Climber climber;

  public RaiseRobot(Climber climber) {
   this.climber = climber;
  }

  
  @Override
  public void initialize() {
    
  }

  
  @Override
  public void execute() {
    climber.raiseRobot();
  }

  
  @Override
  public void end(boolean interrupted) {
    climber.stopRobot();
  }

  
  @Override
  public boolean isFinished() {
    return climber.getRobotHeight() > Constants.Climber.ROBOT_HEIGHT;
  }
}
