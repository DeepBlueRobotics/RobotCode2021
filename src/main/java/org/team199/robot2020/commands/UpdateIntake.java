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
    boolean desired = SmartDashboard.getBoolean("Desired Intake Deployment", false );
    boolean actual = SmartDashboard.getBoolean("Actual Intake Deployment", false );
    
    if (desired && !actual) {
      //run motor to put intake down
      //start running motors that intake balls

    }
    else if (!desired && actual) {
      //run motor to pull intake up
      //stop running motors that intake balls

    }
    else {
      SmartDashboard.putBoolean("Actual Intake Deployment", desired );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //run motors at speed 0

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
