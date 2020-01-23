/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ChangedDesiredIntake extends InstantCommand {
  double buttonPressed;

  public ChangedDesiredIntake(double buttonPressed) {
    this.buttonPressed=buttonPressed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (buttonPressed == SmartDashboard.getNumber("Desired Intake Deployment", 0.0 )) {
      SmartDashboard.putNumber("Desired Intake Deployment", 0.0 );
    }
    else {
      SmartDashboard.putNumber("Desired Intake Deployment", buttonPressed );
    }
  }
}
