package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployClimberShort extends CommandBase {
  private final Climber climber;

  /**
   * Deploys climber up to switch
   * Old needs to be updated to current code
   */
  public DeployClimberShort(Climber climber) {
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
      //In case the winch somehow gets faster than the arm and needs to be stopped before it extends farther than the max
      //this probably won't be necessary thx to equations :)
      if (climber.getWinchHeight() >= Climber.kWinchMaxHeight) {
        climber.runWinch(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //When it reaches the top this stops all of it
    climber.runLift(0);
    //Sets winch to break mode so it doesn't spin freely after arm deploys
    climber.setWinchIdleBrake();
    System.out.println("--ENDED--");
  }

  // returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Checks if arm has reached top
    return climber.getLiftHeight() >= Climber.kLiftShortHeight;
  }
}
