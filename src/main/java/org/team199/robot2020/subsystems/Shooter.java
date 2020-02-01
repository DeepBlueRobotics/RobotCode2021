package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Shooter extends PIDSubsystem {

    private boolean sparkMax = false;
    private final WPI_VictorSPX flywheel1 = MotorControllerFactory.createVictor(Constants.Shooter.VICTOR_FLYWHEEL);
    //uses port 3
    private final CANSparkMax flywheel2 = MotorControllerFactory.createSparkMax(Constants.Shooter.SPARK_FLYWHEEL_1);
    //uses port 2
    private final CANSparkMax flywheel3 = MotorControllerFactory.createSparkMax(Constants.Shooter.SPARK_FLYWHEEL_2);
    //uses port 4
    private final CANPIDController sparkPID = flywheel2.getPIDController();
    private final Encoder encoder = new Encoder(4, 5, false, EncodingType.k1X);
    private final CANEncoder sparkenconder1 = flywheel2.getEncoder();
    private final CANEncoder sparkenconder2 = flywheel3.getEncoder();
    private SimpleMotorFeedforward victorFF = new SimpleMotorFeedforward(Constants.Shooter.KS, Constants.Shooter.KV);
    private SimpleMotorFeedforward sparkFF = new SimpleMotorFeedforward(Constants.Shooter.SPARK_KS, Constants.Shooter.SPARK_KV);
    private double targetSpeed;

    public Shooter() {
        super(new PIDController(Constants.Shooter.KP, Constants.Shooter.KI, Constants.Shooter.KD));
        enable();
        setTargetSpeed(0);
        SmartDashboard.putNumber("Shooter Target Speed", 0);
        SmartDashboard.putNumber("Victor kP", Constants.Shooter.KP);
        SmartDashboard.putNumber("Victor kI", 0);
        SmartDashboard.putNumber("Victor kD", 0);
        SmartDashboard.putNumber("Victor kV", Constants.Shooter.KV);
        SmartDashboard.putNumber("Victor kS", Constants.Shooter.KS);
        SmartDashboard.putNumber("Victor kP", Constants.Shooter.KP);
        SmartDashboard.putNumber("Spark kP", Constants.Shooter.SPARK_KP);
        SmartDashboard.putNumber("Spark kI", 0);
        SmartDashboard.putNumber("Spark kD", 0);
        SmartDashboard.putNumber("Spark kV", Constants.Shooter.SPARK_KV);
        SmartDashboard.putNumber("Spark kS", Constants.Shooter.SPARK_KS);
        flywheel1.enableVoltageCompensation(true);
        flywheel3.follow(flywheel2, true);
        //True makes the second one inverted, otherwise follow will override any inversions and break spark maxes
        flywheel2.setInverted(true);
        //flywheel 3 is slave, uses port 4
        //flywheel 2 is master, uses port 2
        encoder.setDistancePerPulse(-1/8.75);
        encoder.setSamplesToAverage(24);
    }

    public void useOutput(double output, double setpoint) { // set flywheel speed
        if (sparkMax == true) {
            sparkPID.setReference(setpoint, ControlType.kVelocity, 0, sparkFF.calculate(setpoint));
        } else {
            flywheel1.setVoltage(output + victorFF.calculate(setpoint));
        }
    }
    
    public double getMeasurement() { // get current speed
        return sparkenconder1.getVelocity()/60;
    }

    public double getMeasurement2() { // get current speed
        return sparkenconder2.getVelocity()/60;
    }

    public double tempSpark1() {
        return flywheel2.getMotorTemperature();
    }

    public double tempSpark2() {
        return flywheel3.getMotorTemperature();
    }

    public double spark1Current() {
        return flywheel2.getOutputCurrent();
    }

    public double spark2Current() {
        return flywheel3.getOutputCurrent();
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
        return victorFF.ks;
    }

    public double getV() {
        return victorFF.kv;
    }

    public void setSAndV(double kS, double kV) {
        victorFF = new SimpleMotorFeedforward(kS, kV);
        
        // TODO: set spark feedforward
        System.out.println("Created new ff with " + kS + ", " + kV);
    }
}