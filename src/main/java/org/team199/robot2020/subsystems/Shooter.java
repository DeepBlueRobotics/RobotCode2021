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

    public static final double KP = 0.959;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KV = 0.129;
    public static final double KS = 0.0098;
    public static final double SPARK_KP = 0.407;
    public static final double SPARK_KI = 0.0;
    public static final double SPARK_KD = 0.0;
    public static final double SPARK_KV = 0.125;
    public static final double SPARK_KS = 0.0845;
    //TODO create subclass and move PID values there
    private boolean sparkMax = false;
    private final WPI_VictorSPX victorFlywheel = MotorControllerFactory.createVictor(Constants.Drive.kShooterVictor);
    private final CANSparkMax sparkFlywheel1 = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterSparkMaster);
    private final CANSparkMax sparkFlywheel2 = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterSparkSlave);
    private final CANPIDController sparkPID = sparkFlywheel1.getPIDController();
    private final Encoder encoder = new Encoder(4, 5, false, EncodingType.k1X);
    private final CANEncoder sparkenconder1 = sparkFlywheel1.getEncoder();
    private final CANEncoder sparkenconder2 = sparkFlywheel2.getEncoder();
    private SimpleMotorFeedforward victorFF = new SimpleMotorFeedforward(KS, KV);
    private SimpleMotorFeedforward sparkFF = new SimpleMotorFeedforward(SPARK_KS, SPARK_KV);
    private double targetSpeed;

    public Shooter() {
        super(new PIDController(KP, KI, KD));
        enable();
        setTargetSpeed(0);
        sparkFlywheel1.setSmartCurrentLimit(40);
        sparkFlywheel2.setSmartCurrentLimit(40);
        SmartDashboard.putNumber("Shooter Target Speed", 0);
        SmartDashboard.putNumber("Victor kI", 0);
        SmartDashboard.putNumber("Victor kD", 0);
        SmartDashboard.putNumber("Victor kV", KV);
        SmartDashboard.putNumber("Victor kS", KS);
        SmartDashboard.putNumber("Victor kP", KP);
        SmartDashboard.putNumber("Spark kP", SPARK_KP);
        SmartDashboard.putNumber("Spark kI", 0);
        SmartDashboard.putNumber("Spark kD", 0);
        SmartDashboard.putNumber("Spark kV", SPARK_KV);
        SmartDashboard.putNumber("Spark kS", SPARK_KS);
        victorFlywheel.enableVoltageCompensation(true);
        sparkFlywheel2.follow(sparkFlywheel1, true);
        //True makes the second one inverted, otherwise follow will override any inversions and break spark maxes
        sparkFlywheel1.setInverted(true);
        //flywheel 3 is slave, uses port 4
        //flywheel 2 is master, uses port 2
        encoder.setDistancePerPulse(-1/8.75);
        encoder.setSamplesToAverage(24);
    }

    public void useOutput(double output, double setpoint) { // set flywheel speed
        if (sparkMax == true) {
            sparkPID.setReference(setpoint, ControlType.kVelocity, 0, sparkFF.calculate(setpoint));
        } else {
            victorFlywheel.setVoltage(output + victorFF.calculate(setpoint));
        }
    }
    
    public double getMeasurement() { // get current speed
        return sparkenconder1.getVelocity()/60;
    }

    public double getMeasurement2() { // get current speed
        return sparkenconder2.getVelocity()/60;
    }

    public double tempSpark1() {
        return sparkFlywheel1.getMotorTemperature();
    }

    public double tempSpark2() {
        return sparkFlywheel2.getMotorTemperature();
    }

    public double currentSpark1() {
        return sparkFlywheel1.getOutputCurrent();
    }

    public double currentSpark2() {
        return sparkFlywheel2.getOutputCurrent();
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
        System.out.println("Created new ff with " + kS + ", " + kV);
    }
}