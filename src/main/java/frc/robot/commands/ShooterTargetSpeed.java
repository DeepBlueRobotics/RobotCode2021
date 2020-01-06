package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterTargetSpeed extends CommandBase {
    /*
    use limelight to find speed needed to hit outer port
    determine y-value of trajectory at x-value of inner port using limelight 
    determine whether y-value is within inner port 
    put to SmartDashboard
    */
    private double limelightx;
    private double limelighty;
    private double speed;
    private Shooter shooter;

    public ShooterTargetSpeed(Shooter shooter) {
        this.shooter = shooter;
    }

    public ShooterTargetSpeed() {
	}

    //math goes here to get speed from limelights;
    public void TargetSpeed() {
        shooter.setTargetSpeed(speed);
    }
}