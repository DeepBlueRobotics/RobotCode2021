package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Shooter implements Subsystem {
    private final WPI_VictorSPX flywheel = MotorControllerFactory.createVictor(Constants.Drive.FLYWHEEL_MOTOR);
    public static final double SHOOTING_SPEED = 0;
    private double targetSpeed;

    public Shooter() {
        
    }

    public double getCurrentSpeed() {
        return flywheel.get();
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double speed) {
        targetSpeed = speed;
    }

    public void setFlywheelSpeed(double speed){
        flywheel.set(speed);
    }

}