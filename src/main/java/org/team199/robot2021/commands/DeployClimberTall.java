/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployClimberTall extends CommandBase {
  private final Climber climber;

  /**
   * Deploys climber up to switch
   * Old needs to be updated to current code
   */
  public DeployClimberTall(Climber climber) {
    addRequirements(this.climber = climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sends arm up to bar and gives slack from winch to do so
    climber.runLift(Climber.kLiftDeploySpeed);
    //Puts winch into idle mode (coasting) so the arm can lift it
    climber.setWinchIdleCoast();
    System.out.println("--RUNNING--");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //When it reaches the top this stops all of it
    climber.runLift(0);
    System.out.println("--ENDED--");
  }

  // returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Checks if arm has reached top
    return climber.getLiftHeight() >= Climber.kLiftTallHeight;
  }
}
