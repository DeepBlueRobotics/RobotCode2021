package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANPIDController;

import org.team199.lib.MotorControllerFactory;
import org.team199.lib.logging.Log;
import org.team199.robot2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    // private static final double kP = 0.959;
    // private static final double kI = 0.0;
    // private static final double kD = 0.0;
    private static double kV = 0.129;
    private static double kS = 0.105;
    private static final double kP = 0.208;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private double targetSpeed = 0;

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterMaster);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterSlave);
    private final CANPIDController pidController = master.getPIDController();

    public Shooter() {
        master.setSmartCurrentLimit(40);
        slave.setSmartCurrentLimit(40);
        SmartDashboard.putNumber("Shooter Target Speed", 0);
        SmartDashboard.putNumber("Shooter.kP", kP);
        SmartDashboard.putNumber("Shooter.kI", kI);
        SmartDashboard.putNumber("Shooter.kD", kD);
        SmartDashboard.putNumber("Shooter.kV", kV);
        SmartDashboard.putNumber("Shooter.kS", kS);
        
        slave.follow(master, true);
        master.setInverted(false);

        Log.registerDoubleVar("Spark Max Port 2 Speed", () -> master.getEncoder().getVelocity());
        Log.registerDoubleVar("Spark Max Port 4 Speed", () -> slave.getEncoder().getVelocity());
        //Log.setDataLogInterval(10);
    }

    public void periodic()  {
        double p = SmartDashboard.getNumber("Shooter.kP", kP);
        double i = SmartDashboard.getNumber("Shooter.kI", kI);
        double d = SmartDashboard.getNumber("Shooter.kD", kD);

        kV = SmartDashboard.getNumber("Shooter.kV", kV);
        kS = SmartDashboard.getNumber("Shooter.kS", kS);

        if (p != pidController.getP()) pidController.setP(p);
        if (i != pidController.getI()) pidController.setI(i);
        if (d != pidController.getD()) pidController.setD(d);

        //SmartDashboard.putNumber("Shooter Margin of error (RPM)", master.getEncoder().getVelocity() - SmartDashboard.getNumber("Shooter Target Speed", 0));
        // SmartDashboard.putNumber("Temp Spark Max Port 2", master.getMotorTemperature());
        // SmartDashboard.putNumber("Temp Spark Max Port 4", slave.getMotorTemperature());
        // SmartDashboard.putNumber("Current Spark Max Port 2", master.getOutputCurrent());
        // SmartDashboard.putNumber("Current Spark Max Port 4", slave.getOutputCurrent());
        SmartDashboard.putNumber("Speed Spark Max Port 2", master.getEncoder().getVelocity());
        SmartDashboard.putNumber("Speed Spark Max Port 4", slave.getEncoder().getVelocity());

        Log.logData();
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        pidController.setReference(speed, ControlType.kVelocity, 0, calculateFeedForward(speed));
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public double calculateFeedForward(double velocity) {
        return kS * Math.signum(velocity) + kS * velocity;
    }
}