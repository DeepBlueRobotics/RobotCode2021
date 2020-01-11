package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ShooterTargetSpeed;
import frc.robot.commands.EjectCell;

public class Shooter implements Subsystem {
    public static final double SHOOTING_SPEED = 0;
    private double targetSpeed;
    private double currentSpeed;
    public Shooter() {
        
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double speed) {
        targetSpeed = speed;
    }
}