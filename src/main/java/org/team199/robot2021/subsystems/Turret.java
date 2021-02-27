package org.team199.robot2021.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;

public class Turret extends SubsystemBase {
    
    private final DigitalInput homeSensor = new DigitalInput(Constants.Drive.kTurretHomeSensor);
    private final DigitalInput counterClockwiseLimit = new DigitalInput(Constants.Drive.kTurretCounterclockwiseLimit);
    private final DigitalInput clockwiseLimit = new DigitalInput(Constants.Drive.kTurretClockwiseLimit);
    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Drive.kTurretMotor);
    private final CANEncoder encoder = motor.getEncoder();
    private final double gearing = 1; //TODO: Set Correct Value

    public Turret() {
        encoder.setPositionConversionFactor(gearing * 180 / (42 * 2 * Math.PI));
    }

    @Override
    public void periodic() {
        double speed = motor.get();
        if(speed == 0) {
            return;
        }
        if(limited(speed)) {
            motor.set(0);
        }
    }

    public void turnCounterclockwise() {
        motor.set(1);
    }

    public void turnClockwise() {
        motor.set(-1);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }

    public double getSpeed() {
        return motor.get();
    }

    public boolean isAtCounterclockwiseLimit() {
        return counterClockwiseLimit.get();
    }

    public boolean isAtClockwiseLimit() {
        return clockwiseLimit.get();
    }

    public boolean isAtHome() {
        return homeSensor.get();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetPosition() {
        encoder.setPosition(0);
    }

    public boolean limited(double speed) {
        return (Math.signum(speed) == 1 ? counterClockwiseLimit : clockwiseLimit).get();
    }

}
