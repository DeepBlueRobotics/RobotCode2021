package org.team199.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber extends SubsystemBase {
    private static final double kLiftConversionFactor =6.0/40 * Math.PI * 3; // TODO: confirm numbers (inches)
    private static final double kWinchConversionFactor = 1.0/9 * Math.PI; // TODO: confirm numbers (inches)

    // TODO: find good values and then set to final
    public static double kLiftDeploySpeed = 0.9; // TODO: set correct speed
    public static double kWinchDeploySpeed = 0.5; // TODO: set correct speed
    public static double kLiftKeepSpeed = 0; // TODO: set correct speed
    public static double kLiftRetractSpeed = -0.5; // TODO: set correct speed
    public static double kWinchRetractSpeed = 0.8; // TODO: set correct speed
    public static double kLiftAdjustSpeed = 0.1; // TODO: set correct speed
    
    public static final double kLiftHeight = 73.5;
    public static final double kWinchEndHeight = 76.2;
    public static final double kWinchStartHeight = -90;

    private final CANSparkMax liftMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kClimberLift);
    private final CANSparkMax winchMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kClimberWinch);

    public Climber(){
        liftMotor.getEncoder().setPositionConversionFactor(kLiftConversionFactor);
        winchMotor.getEncoder().setPositionConversionFactor(kWinchConversionFactor);
        liftMotor.getEncoder().setPosition(0);
        winchMotor.getEncoder().setPosition(kWinchStartHeight);

        SmartDashboard.putNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        SmartDashboard.putNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        SmartDashboard.putNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        SmartDashboard.putNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        SmartDashboard.putNumber("Climber.kWinchRetractSpeed", kWinchRetractSpeed);
        SmartDashboard.putNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
    }

    public void periodic()  {
        kLiftDeploySpeed = SmartDashboard.getNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        kWinchDeploySpeed = SmartDashboard.getNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        kLiftKeepSpeed = SmartDashboard.getNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        kLiftRetractSpeed = SmartDashboard.getNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        kWinchRetractSpeed = SmartDashboard.getNumber("Climber.kWinchRetractSpeed", kWinchRetractSpeed);
        kLiftAdjustSpeed = SmartDashboard.getNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
        SmartDashboard.putNumber("Lift Position", getLiftHeight());
        SmartDashboard.putNumber("Winch Position", getWinchHeight());
        SmartDashboard.putNumber("Lift Speed", getLiftSpeed());
        SmartDashboard.putNumber("Winch Speed", getWinchSpeed());
    }

    public void runLift(double speed) {
        liftMotor.set(speed);
    }

    public void runWinch(double speed) {
        System.out.println("HEYY WINCH IS RUNNING AT SPEED " + winchMotor.getEncoder().getVelocity());
        winchMotor.set(Math.abs(speed));
    }

    public double getLiftHeight() {
        return liftMotor.getEncoder().getPosition();
    }

    public double getWinchHeight() {
        return winchMotor.getEncoder().getPosition();
    }

    public double getLiftSpeed() {
        return liftMotor.getEncoder().getVelocity();
    }

    public double getWinchSpeed() {
        return winchMotor.getEncoder().getVelocity();
    }
}