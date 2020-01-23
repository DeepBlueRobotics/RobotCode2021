/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateIntake extends CommandBase {
  /**
   * Creates a new UpdateIntake.
   */
  Intake intake;
  public UpdateIntake(Intake intake) {
    this.intake=intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desired = SmartDashboard.getNumber("Desired Intake Deployment", 0.0 );
    double actual = SmartDashboard.getNumber("Actual Intake Deployment", 0.0 );
    
    if (desired == 1 && actual != 1) {
      intake.intake();
    }
    else if (desired == 0 && actual != 0) {
      intake.stopIntake();
    }
    else if (desired == -1 && actual != -1){
      intake.outtake();
    }
    SmartDashboard.putNumber("Actual Intake Deployment", desired );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
