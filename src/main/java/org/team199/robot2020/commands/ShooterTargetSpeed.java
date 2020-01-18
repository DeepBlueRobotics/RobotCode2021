package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTargetSpeed extends CommandBase {
  Shooter shooter;
  PIDController pid;
  

  public ShooterTargetSpeed(Shooter shooter) {
    this.shooter = shooter;
    pid = new PIDController(0, 0, 0);
  }

  public void execute() {
    /*
   Always set the flywheel to the current desired speed using PID
   Update current velocity measurement
   */
    double speed = shooter.getTargetSpeed(); //makes Kevin #4 feel better
    shooter.setFlywheelSpeed(speed);
    SmartDashboard.putNumber("Shooter Speed", shooter.getCurrentSpeed());
  }

  public boolean isFinished() {
    return false;
  }
}