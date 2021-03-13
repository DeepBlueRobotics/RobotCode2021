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
  private double voltage;


  public LowerArm(Climber climber) {
    addRequirements(this.climber = climber);
    voltage = climber.getVoltage();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runLift(climber.kLiftLowerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop lift and winch when arm returns to robot
    climber.runLift(0);
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

