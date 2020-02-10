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
    
    double speed = SmartDashboard.getNumber("Shooter Target Speed", 0); //makes Kevin #4 feel better
    if (shooter.getTargetSpeed() != speed) shooter.setSpeed(speed);

    //SmartDashboard.putNumber("Limelight Distance Adjustment", limelight.distanceAssist());
    //SmartDashboard.putNumber("Limelight Angle Adjustment", limelight.steeringAssist());
    //Limelight.java already puts horizontal and vertical offset numbers to Smartdashboard
  }

  public boolean isFinished() {
    return false;
  }
}