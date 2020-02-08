package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import org.team199.lib.Limelight;

public class ShooterSpeedControl extends CommandBase {
  Shooter shooter;
  Timer timer;
  Limelight limelight;

  public ShooterSpeedControl(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    timer = new Timer();
  }

  public void initialize() {
    timer.start();
  }

  public void execute() {
    /*
   Always set the flywheel to the current desired speed using PID
   Update current velocity measurement
   */
    readSmartDashboard();
    
    double speed = SmartDashboard.getNumber("Shooter Target Speed", 0); //makes Kevin #4 feel better
    shooter.setSetpoint(speed);
    SmartDashboard.putNumber("Shooter Distance", shooter.getCurrentDistance());

    SmartDashboard.putNumber("Margin of error Port 2", SmartDashboard.getNumber("Spark Port 2 Speed", 0) - SmartDashboard.getNumber("Shooter Target Speed", 0));
    SmartDashboard.putNumber("Margin of error Port 4", SmartDashboard.getNumber("Spark Port 4 Speed", 0) - SmartDashboard.getNumber("Shooter Target Speed", 0));
    SmartDashboard.putNumber("Temp Spark Max Port 2", shooter.tempSpark1());
    SmartDashboard.putNumber("Temp Spark Max Port 4", shooter.tempSpark2());
    SmartDashboard.putNumber("Current Spark Max Port 2", shooter.currentSpark1());
    SmartDashboard.putNumber("Current Spark Max Port 4", shooter.currentSpark2());

    SmartDashboard.putNumber("Limelight Distance Adjustment", limelight.distanceAssist());
    SmartDashboard.putNumber("Limelight Angle Adjustment", limelight.steeringAssist());
    //Limelight.java already puts horizontal and vertical offset numbers to Smartdashboard

    if (timer.get() >= 0.1) {
      SmartDashboard.putNumber("Spark Port 2 Speed", shooter.getMeasurement());
      SmartDashboard.putNumber("Spark Port 4 Speed", shooter.getMeasurement2());

      timer.reset();
    }
  }

  public void readSmartDashboard() {
    if (SmartDashboard.getNumber("Shooter kP", 0) != shooter.getP()) {
      shooter.setP(SmartDashboard.getNumber("Shooter kP", 0));
    }
    if (SmartDashboard.getNumber("Shooter kI", 0) != shooter.getI()) {
      shooter.setI(SmartDashboard.getNumber("Shooter kI", 0));
    }
    if (SmartDashboard.getNumber("Shooter kD", 0) != shooter.getD()) {
      shooter.setD(SmartDashboard.getNumber("Shooter kD", 0));
    }
    if (SmartDashboard.getNumber("Shooter kV", 0) != shooter.getV() || 
        SmartDashboard.getNumber("Shooter kS", 0) != shooter.getS()) {
      shooter.setSAndV(SmartDashboard.getNumber("Shooter kS", 0), SmartDashboard.getNumber("Shooter kV", 0));
    }
    if (shooter.getSparkMaxStatus() == true) {
      SmartDashboard.putString("Motor status", "The shooter is using the spark max");
    } else {
      SmartDashboard.putString("Motor status", "The shooter is using the Victor SPX");
    }
    shooter.setSparkMaxStatus(SmartDashboard.getBoolean("Motor status", true));
  }

  public boolean isFinished() {
    return false;
  }
}