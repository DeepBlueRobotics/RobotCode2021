package org.team199.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANPIDController;

import org.team199.lib.Limelight;
import org.team199.lib.LinearInterpolation;
import org.team199.lib.MotorControllerFactory;
import org.team199.lib.logging.Log;
import org.team199.robot2020.Constants;
import org.team199.robot2020.RobotContainer.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;
    private static double lkP;
    private static double lkI;
    private static double lkD;

    private double kTargetSpeed = 4200;
    private final double speedOffset = 100;

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterMaster);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.Drive.kShooterSlave);
    private final CANPIDController pidController = master.getPIDController();

    private final Drivetrain drivetrain;
    private final Limelight lime;
    private final LinearInterpolation linearInterpol;
    private final Target target;

    // TODO: Shooter should not have to require this many objects. Rework the LinearInterpolation code.
    public Shooter(Drivetrain drivetrain, Limelight lime, LinearInterpolation linearInterpol, Target target) {
        this.drivetrain = drivetrain;
        this.lime = lime;
        this.linearInterpol = linearInterpol;
        this.target = target;

        lkP = lime.getPIDController().getP();
        lkI = lime.getPIDController().getI();
        lkD = lime.getPIDController().getD();

        master.setSmartCurrentLimit(40);
        slave.setSmartCurrentLimit(40);

        SmartDashboard.putNumber("Shooter.kTargetSpeed", kTargetSpeed);
        SmartDashboard.putNumber("Shooter.kP", kP);
        SmartDashboard.putNumber("Shooter.kI", kI);
        SmartDashboard.putNumber("Shooter.kD", kD);
        SmartDashboard.putNumber("Shooter.kV", kV);
        SmartDashboard.putNumber("Shooter.kS", kS);
        SmartDashboard.putNumber("Limelight.kP", lime.getPIDController().getP());
        SmartDashboard.putNumber("Limelight.kI", lime.getPIDController().getI());
        SmartDashboard.putNumber("Limelight.kD", lime.getPIDController().getD());
        
        slave.follow(master, true);
        master.setInverted(true);

        Log.registerDoubleVar("Spark Max Port 2 Speed", () -> master.getEncoder().getVelocity());
        Log.registerDoubleVar("Spark Max Port 4 Speed", () -> slave.getEncoder().getVelocity());
    }

    public void periodic()  {
        double p = SmartDashboard.getNumber("Shooter.kP", kP);
        double i = SmartDashboard.getNumber("Shooter.kI", kI);
        double d = SmartDashboard.getNumber("Shooter.kD", kD);
        double lp = SmartDashboard.getNumber("Limelight.kP", lkP);
        double li = SmartDashboard.getNumber("Limelight.kI", lkI);
        double ld = SmartDashboard.getNumber("Limelight.kD", lkD);

        kV = SmartDashboard.getNumber("Shooter.kV", kV);
        kS = SmartDashboard.getNumber("Shooter.kS", kS);
        SmartDashboard.putNumber("Shooter.kTargetSpeed", 
            linearInterpol.calculate(target.pos.getDistance(drivetrain.getOdometry().getPoseMeters().getTranslation()))
        );
        setSpeed(SmartDashboard.getNumber("Shooter.kTargetSpeed", kTargetSpeed));

        if (p != pidController.getP()) pidController.setP(p);
        if (i != pidController.getI()) pidController.setI(i);
        if (d != pidController.getD()) pidController.setD(d);
        if (lp != lime.getPIDController().getP()) lime.getPIDController().setP(lp);
        if (li != lime.getPIDController().getI()) lime.getPIDController().setI(li);
        if (ld != lime.getPIDController().getD()) lime.getPIDController().setD(ld);
        pidController.setReference(getTargetSpeed(), ControlType.kVelocity, 0, calculateFeedForward(getTargetSpeed()));
        
        SmartDashboard.putNumber("Speed Spark Max Port 2", master.getEncoder().getVelocity());
        SmartDashboard.putNumber("Speed Spark Max Port 4", slave.getEncoder().getVelocity());
    }

    public void setSpeed(double speed) {
        kTargetSpeed = speed;
    }

    public double getTargetSpeed() {
        return kTargetSpeed;
    }

    public boolean isAtTargetSpeed() {
        return (master.getEncoder().getVelocity() > (kTargetSpeed - speedOffset));
    }

    public double calculateFeedForward(double velocity) {
        return kS * Math.signum(velocity) + kV * velocity;
    }
}