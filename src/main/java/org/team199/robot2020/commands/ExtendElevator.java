/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import motors

public class ExtendElevator extends CommandBase {
  //initialize motors

  public ExtendElevator() {
    //control the motor with a button on the remote
    //when the button is pressed the motor raises the lift and then slowly starts to lower
  
  }

  
  @Override
  public void initialize() {     
  }

  @Override
  public void execute() {
    //make motor run
  }

  
  @Override
  public void end(final boolean interrupted) {
    //motors stop running
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
