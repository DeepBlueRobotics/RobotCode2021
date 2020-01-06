package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem {
    private double targetSpeed;
    public Shooter() {
        
    }

    public void setTargetSpeed() {
        // get speed necessary to hit target from limelight
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }
}