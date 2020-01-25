/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import org.team199.robot2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ToggleIntake extends InstantCommand {
  private Intake intake;
  public ToggleIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isRunning()) {
      intake.deploy();
      intake.intake();
    } else {
      intake.retract();
      intake.stop();
    }
  }
}
