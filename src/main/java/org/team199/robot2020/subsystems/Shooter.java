package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
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