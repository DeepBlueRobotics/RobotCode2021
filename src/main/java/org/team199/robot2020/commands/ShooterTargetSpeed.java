package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class ShooterTargetSpeed extends CommandBase {
  Shooter shooter;
  Timer timer;

  public ShooterTargetSpeed(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    timer = new Timer();
  }

  public void initialize(){
    timer.start();
  }

  public void execute() {
    /*
   Always set the flywheel to the current desired speed using PID
   Update current velocity measurement
   */
    if (SmartDashboard.getNumber("Shooter kP", 0) != shooter.getP()) {
      shooter.setP(SmartDashboard.getNumber("Shooter kP", 0));
    }
    double speed = SmartDashboard.getNumber("Shooter Target Speed", 0); //makes Kevin #4 feel better
    shooter.setSetpoint(speed);
    SmartDashboard.putNumber("Shooter Distance", shooter.getCurrentDistance());

    if (timer.get() >= 0.1){
      SmartDashboard.putNumber("Shooter Speed", shooter.getMeasurement());
      timer.reset();
    }
  }

  public boolean isFinished() {
    return false;
  }
}