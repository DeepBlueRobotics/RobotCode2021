package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    double speed = shooter.getTargetSpeed(); //makes Kevin #4 feel better
    shooter.setSetpoint(speed);
    SmartDashboard.putNumber("Shooter Speed", shooter.getMeasurement());
  }

  public boolean isFinished() {
    return false;
  }
}