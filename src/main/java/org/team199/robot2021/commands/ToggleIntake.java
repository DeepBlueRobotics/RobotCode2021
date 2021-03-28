/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntake extends CommandBase {
  /**
   * Creates a new ToggleIntake.
   */
  private Intake intake;
  private Timer timer;
  private boolean beingDeployed;

  public ToggleIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = intake);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isDeployed()) {
      intake.stop();
      intake.retract();
      beingDeployed = false;
    } else {
      intake.doTheFlop();
      timer.start();
      beingDeployed = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(beingDeployed){
      intake.intake();
      timer.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(Intake.kTimeToDeploy) || !beingDeployed;
  }
}
