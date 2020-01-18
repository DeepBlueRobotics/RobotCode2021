package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private final WPI_VictorSPX flywheel = MotorControllerFactory.createVictor(Constants.Drive.FLYWHEEL_MOTOR);
    private final Encoder encoder = new Encoder(new DigitalInput(0), new DigitalInput(1));
    public static final double SHOOTING_SPEED = 0;
    private double targetSpeed;

    public Shooter() {
        SmartDashboard.putNumber("Shooter Target Speed", 0);
        encoder.setDistancePerPulse(Math.PI * 5 / 256);
    }

    public double getCurrentSpeed() {
        return encoder.getRate();
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