package org.team199.robot2020.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber extends SubsystemBase {
    private final CANSparkMax liftMotor = MotorControllerFactory.createSparkMax(Constants.Climber.LIFT_MOTOR);
    private final CANSparkMax winchMotor = MotorControllerFactory.createSparkMax(Constants.Climber.WINCH_MOTOR);

    public Climber(){
        liftMotor.getEncoder().setPositionConversionFactor(Constants.Climber.LIFT_CONVERSION_FACTOR);
        winchMotor.getEncoder().setPositionConversionFactor(Constants.Climber.WINCH_CONVERSION_FACTOR);
        winchMotor.getEncoder().setPosition(Constants.Climber.WINCH_START_HEIGHT);
    }

    public void runLift(double speed) {
        liftMotor.set(speed);
    }

    public void runWinch(double speed) {
        winchMotor.set(Math.abs(speed));
    }

    public double getLiftHeight() {
        return liftMotor.getEncoder().getPosition();
    }

    public double getWinchHeight() {
        return winchMotor.getEncoder().getPosition();
    }
}