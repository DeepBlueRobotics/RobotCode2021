package org.team199.robot2020.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // TODO: find good values and then set to final
    private static double kIntakeSpeed = 0.5;
    private static double kSlowSpeed = -0.05;

    private final CANSparkMax rollerMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kIntakeRoller);
    private final DoubleSolenoid intakePistons = new DoubleSolenoid(Constants.Drive.kIntakePistons[0], Constants.Drive.kIntakePistons[1]);

    private boolean deployed = false;

    /**
     * Vectored intake that rolls balls through the bumper gap and into feeder.
     */
    public Intake() {
        rollerMotor.setInverted(false);
        rollerMotor.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        SmartDashboard.putNumber("Intake.kSlowSpeed", kSlowSpeed);
    }

    public void periodic() {
        kIntakeSpeed = SmartDashboard.getNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        kSlowSpeed = SmartDashboard.getNumber("Intake.kSlowSpeed", kSlowSpeed);
    }

    public void slow() {
        rollerMotor.set(kSlowSpeed);
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
        intakePistons.set(DoubleSolenoid.Value.kForward);
        deployed = true;
    }

    public void retract() {
        intakePistons.set(DoubleSolenoid.Value.kReverse);
        deployed = false;
    }

    // for if we want to try making our intake less rigid
    public void doTheFlop() {
        intakePistons.set(DoubleSolenoid.Value.kOff);
    }

    public boolean isDeployed() {
        return deployed;
    }
}