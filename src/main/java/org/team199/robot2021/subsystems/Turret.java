package org.team199.robot2021.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.team199.robot2021.Constants;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;

public class Turret extends SubsystemBase {
    
    private final DigitalInput homeSensor = new DigitalInput(Constants.DrivePorts.kTurretHomeSensor);
    private final DigitalInput limitSensor = new DigitalInput(Constants.DrivePorts.kTurretLimitSensor);
    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kTurretMotor);
    private final CANEncoder encoder = motor.getEncoder();
    private final double gearing = 1/50D;

    private double simPos = 10;
    private long simLastUpdate = -1;
    private double simLastSpeed = 0;
    private DIOSim simHomeSensor;
    private DIOSim simLimitSensor;

    public Turret() {
        encoder.setPositionConversionFactor(gearing * 180 / (42 * 2 * Math.PI));
        if(RobotBase.isSimulation()) {
            simHomeSensor = new DIOSim(homeSensor);
            simLimitSensor = new DIOSim(limitSensor);

            SmartDashboard.putNumber("Target Position", 0.0D);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position", encoder.getPosition());
        double speed = motor.get();
        if(speed == 0) {
            if(isAtLimit() && Math.abs(getPosition()) > 100) {
                motor.set(-Math.copySign(0.1, getPosition()));
            }
            return;
        }
        if(limited(speed)) {
            motor.set(0);
        }
    }

    public void turnCounterclockwise() {
        if(!limited(0.25)) {
            motor.set(0.25);
        }
    }

    public void turnClockwise() {
        if(!limited(-0.25)) {
            motor.set(-0.25);
        }
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

    public boolean isAtLimit() {
        return limitSensor.get();
    }

    public boolean isAtHome() {
        return homeSensor.get();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setPosition(double pos) {
        encoder.setPosition(pos);
    }

    public boolean limited(double speed) {
        return Math.signum(speed) == Math.signum(getPosition()) && (isAtLimit() || Math.abs(getPosition()) >= 180);
    }

    @Override
    public void simulationPeriodic() {
        {
            long currentTimeMillis = System.currentTimeMillis();
            if(simLastUpdate != -1) {
                double deltaPos = simLastSpeed * 360D * ((75D / 60D) / (1000D/(currentTimeMillis - simLastUpdate)));
                simPos += deltaPos;
                encoder.setPosition(encoder.getPosition()+deltaPos);
                SmartDashboard.putNumber("Simulated Turret Position", simPos);
                simHomeSensor.setValue(isInRange(simPos, 0, 10));
                simLimitSensor.setValue(isInRange(Math.abs(simPos), 180, 10));
            }
            simLastUpdate = currentTimeMillis;
            simLastSpeed = motor.get();
        }

        {
            double diff = simPos - SmartDashboard.getNumber("Target Position", 0.0D);
            boolean tv = Math.abs(diff) <= 27.0;
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").setDouble(tv ? diff : 0.0D);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").setDouble(tv ? 1.0D : 0.0D);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").setDouble(1.0D);
        }
    }

    public static boolean isInRange(double d, double val, double delta) {
        return Math.abs(val - d) <= delta;
    }

}
