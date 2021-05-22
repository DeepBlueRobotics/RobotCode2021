/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerArm extends CommandBase {
  private final Climber climber;
  //change voltage to isButtonPressed
  private boolean hookAttached;
  


  public LowerArm(Climber climber) {
    addRequirements(this.climber = climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){ 
    if(climber.isFinishedDeploying == true){
      hookAttached = climber.isHookAttached();
      if (!hookAttached) {
        climber.runLift(climber.kLiftLowerPostHookSpeed);
    }    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make while loop that constantly checks if isbuttonpressed, cancel this command if it ever becomes false
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop lift and winch when arm returns to robot
    if(climber.isFinishedDeploying == true){
      climber.runLift(0);
      System.out.println("--ENDED--");     
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(climber.isFinishedDeploying == false){
      return true;
    } else{
    //checks if arm has returned to robot
    return (climber.getLiftHeight() <= Climber.kLiftLowerHeight) || hookAttached;
    } 
  }  
}

