package org.team199.robot2021.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;

public class Turret extends SubsystemBase {
    
    private final DigitalInput homeSensor = new DigitalInput(0);
    private final DigitalInput counterClockwiseLimit = new DigitalInput(0);
    private final DigitalInput clockwiseLimit = new DigitalInput(0);
    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(0);
    private final CANEncoder encoder = motor.getEncoder();

    public Turret() {
        encoder.setPositionConversionFactor(0);
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
