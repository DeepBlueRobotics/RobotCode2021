package org.team199.robot2021.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    private final DigitalInput homeSensor = new DigitalInput(0);
    private final DigitalInput counterClockwiseLimit = new DigitalInput(0);
    private final DigitalInput clockwiseLimit = new DigitalInput(0);
    private final PlaceholderMotor motor = null;
    private final PlaceholderEncoder encoder = null;

    public Turret() {
        encoder.setDistancePerPulse(0);
    }

    @Override
    public void periodic() {
        double speed = motor.get();
        if(speed == 0) {
            return;
        }
        if((Math.signum(speed) == 1 ? counterClockwiseLimit : clockwiseLimit).get()) {
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
        return encoder.getDistance();
    }

    public void resetPosition() {
        encoder.reset();
    }

    interface PlaceholderMotor {
        void set(double speed);
        double get();
    }

    interface PlaceholderEncoder {
        double getDistance();
        void setDistancePerPulse(double distancePerPulse);
        void reset();
    }

}
