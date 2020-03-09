package org.team199.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber extends SubsystemBase {
    private static final double kLiftConversionFactor = 6.0/40 * Math.PI * 3; // TODO: confirm numbers (inches)
    private static final double kWinchConversionFactor = 1.0/9 * Math.PI; // TODO: confirm numbers (inches)

    // TODO: find good values and then set to final
    public static double kLiftDeploySpeed = 0.3; // TODO: set correct speed
    public static double kWinchDeploySpeed = 1; // TODO: set correct speed
    public static double kLiftKeepSpeed = 0.06; // TODO: set correct speed
    public static double kLiftRetractSpeed = -0.3; // TODO: set correct speed
    public static double kWinchRetractSpeed = 0.6; // TODO: set correct speed
    public static double kLiftAdjustSpeed = 0.2; // TODO: set correct speed
    public static double kWinchAdjustSpeed = 0.2;

    public static final double kLiftHeight = 87;
    public static final double kWinchEndHeight = 30;
    public static final double kWinchStartHeight = -80;
    private final CANSparkMax liftMotor = MotorControllerFactory.createSparkMax(Constants.Ports.kClimberLift);
    private final CANSparkMax winchMotor = MotorControllerFactory.createSparkMax(Constants.Ports.kClimberWinch);
    private final CANEncoder liftEnc = liftMotor.getEncoder();
    private final CANEncoder winchEnc = winchMotor.getEncoder();

    public Climber(){
        liftEnc.setPositionConversionFactor(kLiftConversionFactor);
        winchEnc.setPositionConversionFactor(kWinchConversionFactor);
        liftEnc.setVelocityConversionFactor(kLiftConversionFactor / 60);
        winchEnc.setVelocityConversionFactor(kWinchConversionFactor / 60);
        liftEnc.setPosition(0);
        winchEnc.setPosition(kWinchStartHeight);
        winchMotor.setIdleMode(IdleMode.kCoast);
        liftMotor.setSmartCurrentLimit(30);

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
        System.out.println("CARRY ME BOII IS SPEED " + liftMotor.getEncoder().getVelocity());
        liftMotor.set(speed);
        if (liftMotor.getEncoder().getVelocity() == 0){
            System.out.println("UNWIND WHEN DONE TESTING!!!");
        }
    }

    public void runWinch(double speed) {
        System.out.println("HEYY WINCH IS RUNNING AT SPEED " + winchMotor.getEncoder().getVelocity());
        winchMotor.set(Math.abs(speed));
    }

    public double getLiftHeight() {
        return liftEnc.getPosition();
    }

    public double getWinchHeight() {
        return winchEnc.getPosition();
    }

    public double getLiftSpeed() {
        return liftEnc.getVelocity();
    }

    public double getWinchSpeed() {
        return winchEnc.getVelocity();
    }
}