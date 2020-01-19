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
    addRequirements(shooter);
    pid = new PIDController(0, 0, 0);
  }

  public void execute() {
    /*
   Always set the flywheel to the current desired speed using PID
   Update current velocity measurement
   */
    double speed = SmartDashboard.getNumber("Shooter Target Speed", 0); //makes Kevin #4 feel better
    shooter.setFlywheelSpeed(speed);
    SmartDashboard.putNumber("Shooter Distance", shooter.getCurrentDistance());
    SmartDashboard.putNumber("Shooter Speed", java.lang.Math.round(shooter.getCurrentSpeed()));
  }

  public boolean isFinished() {
    return false;
  }
}