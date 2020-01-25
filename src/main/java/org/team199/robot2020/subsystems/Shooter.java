package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.revrobotics.CANPIDController;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Shooter extends PIDSubsystem {

    private boolean sparkMax = false;
    private final WPI_VictorSPX flywheel1 = MotorControllerFactory.createVictor(Constants.Shooter.FLYWHEEL_MOTOR);
    private final CANSparkMax flywheel2 = MotorControllerFactory.createSparkMax(Constants.Shooter.FLYWHEEL_MOTOR);
    private final CANSparkMax flywheel3 = MotorControllerFactory.createSparkMax(Constants.Shooter.FLYWHEEL_MOTOR);
    private final CANPIDController sparkPID = flywheel2.getPIDController();
    
    private final Encoder encoder = new Encoder(4, 5, false, EncodingType.k1X);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(-3.42857142857, 6.0/35);
    private double targetSpeed;

    public Shooter() {
        super(new PIDController(Constants.Shooter.KP, Constants.Shooter.KI, Constants.Shooter.KD));
        enable();
        setTargetSpeed(0);
        SmartDashboard.putNumber("Shooter Target Speed", 0);
        SmartDashboard.putNumber("Shooter kP", Constants.Shooter.KP);
        SmartDashboard.putNumber("Shooter kI", 0);
        SmartDashboard.putNumber("Shooter kD", 0);
        SmartDashboard.putNumber("Shooter kV", Constants.Shooter.KV);
        SmartDashboard.putNumber("Shooter kS", Constants.Shooter.KS);
        SmartDashboard.putNumber("Margin of error", SmartDashboard.getNumber("Shooter Speed", 0) - SmartDashboard.getNumber("Shooter Target Speed", 0));
        flywheel1.enableVoltageCompensation(true);
        flywheel3.setInverted(true);
        flywheel3.follow(flywheel2);
        encoder.setDistancePerPulse(-1/8.75);
        encoder.setSamplesToAverage(24);
    }

    public void useOutput(double output, double setpoint) { // set flywheel speed
        if (sparkMax == true) {
            sparkPID.setReference(setpoint, ControlType.kVelocity);
        } else {
            flywheel1.setVoltage(output + ff.calculate(setpoint));
        }
    }
    
    public double getMeasurement() { // get current speed
        return encoder.getRate();
    }

    public double getCurrentDistance() {
        return encoder.getDistance();
    }

    public boolean getSparkMaxStatus() {
        return sparkMax;
    }

    public void setSparkMaxStatus(boolean sparkMax) {
        this.sparkMax = sparkMax;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double speed) {
        setSetpoint(speed);
        targetSpeed = speed;
    }

    public double getP() {
        return getController().getP();
    }

    public void setP(double kP) {
        getController().setP(kP);
        sparkPID.setP(kP);
    }

    public double getI() {
        return getController().getI();
    }

    public void setI(double kI) {
        getController().setI(kI);
        sparkPID.setI(kI);
    }

    public double getD() {
        return getController().getD();
    }

    public void setD(double kD) {
        getController().setD(kD);
        sparkPID.setD(kD);
    }

    public double getS() {
        return ff.ks;
    }

    public double getV() {
        return ff.kv;
    }

    public void setSAndV(double kS, double kV) {
        ff = new SimpleMotorFeedforward(kS, kV);
        // TODO: set spark feedforward
        System.out.println("Created new ff with " + kS + ", " + kV);
    }
}