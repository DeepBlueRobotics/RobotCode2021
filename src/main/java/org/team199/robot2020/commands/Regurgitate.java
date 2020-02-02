/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Regurgitate extends CommandBase {
  private Intake intake;
  private Feeder feeder;
  
  /**
   * Regurgitates the balls out of the feeder (and intake if it's deployed)
   */
  public Regurgitate(Intake intake, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = intake, this.feeder = feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.runBackward();
    if (intake.isDeployed()) {
      intake.outtake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    if (intake.isDeployed()) {
      intake.intake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
