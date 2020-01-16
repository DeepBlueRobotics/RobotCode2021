package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Shooter;

public class ShooterTargetSpeed extends CommandBase {
  Shooter shooter;

  public ShooterTargetSpeed(Shooter shooter) {
    this.shooter = shooter;
  }

  public void execute() {
    /*
   Always set the flywheel to the current desired speed using PID
   Update current velocity measurement
   */
  }

  public boolean isFinished() {
    return false;
  }
}