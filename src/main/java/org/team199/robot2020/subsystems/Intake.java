package org.team199.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum States {

    }
    // TODO: find good values and then set to final
    private static double kIntakeSpeed = 0.4;
    private static double kReverseSpeed = -0.1;
    private static double kOutputLimit = 0.3;
    public double targetEncoderDist = 5.0;   // TODO: Figure out the correct value.
    public boolean encoderReset = false;
    public boolean cell5Entered = false;

    private final CANSparkMax rollerMotor = MotorControllerFactory.createSparkMax(Constants.Ports.kIntakeRoller);
    private final CANEncoder rollerEncoder = rollerMotor.getEncoder();
    private final DoubleSolenoid intakePistons1 = new DoubleSolenoid(Constants.Ports.kIntakePistons[0], Constants.Ports.kIntakePistons[1]);
    private final DoubleSolenoid intakePistons2 = new DoubleSolenoid(Constants.Ports.kIntakePistons[2], Constants.Ports.kIntakePistons[3]);

    private boolean deployed = false;

    /**
     * Vectored intake that rolls balls through the bumper gap and into feeder.
     */
    public Intake() {
        rollerMotor.setInverted(false);
        rollerMotor.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        SmartDashboard.putNumber("Intake.kReverseSpeed", kReverseSpeed);
        SmartDashboard.putNumber("Intake.kOutputLimit", kOutputLimit);
    }

    public void periodic() {
        kIntakeSpeed = SmartDashboard.getNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        kReverseSpeed = SmartDashboard.getNumber("Intake.kReverseSpeed", kReverseSpeed);
        kOutputLimit = SmartDashboard.getNumber("Intake.kOutputLimit", kOutputLimit);
        // SparkMax does not allow you to set an output limit so it is periodically checked whether the intake speeds conform to the limit.
        kIntakeSpeed = Math.signum(kIntakeSpeed) * Math.min(kOutputLimit, Math.abs(kIntakeSpeed));
        kReverseSpeed = Math.signum(kReverseSpeed) * Math.min(kOutputLimit, Math.abs(kReverseSpeed));
        SmartDashboard.putNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        SmartDashboard.putNumber("Intake.kReverseSpeed", kReverseSpeed);
    }

    public void reverse() {
        rollerMotor.set(kReverseSpeed);
    }

    public void intake() {
        rollerMotor.set(kIntakeSpeed);
    }

    public void outtake() {
        rollerMotor.set(-kIntakeSpeed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void deploy() {
        intakePistons1.set(DoubleSolenoid.Value.kReverse);
        intakePistons2.set(DoubleSolenoid.Value.kForward);
        deployed = true;
    }

    public void retract() {
        intakePistons1.set(DoubleSolenoid.Value.kForward);
        intakePistons2.set(DoubleSolenoid.Value.kReverse);
        deployed = false;
    }

    // for if we want to try making our intake less rigid
    public void doTheFlop() {
        intakePistons1.set(DoubleSolenoid.Value.kReverse);
        intakePistons2.set(DoubleSolenoid.Value.kReverse);
        deployed = true;
    }

    public boolean isDeployed() {
        return deployed;
    }

    public double getEncoderDistance() {
        return rollerEncoder.getPosition();
    }

    public void resetEncoder() {
        rollerEncoder.setPosition(0.0);
    }
}