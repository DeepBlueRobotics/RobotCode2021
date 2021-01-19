package org.team199.robot2021.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.lib.MotorControllerFactory;
import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // TODO: find good values and then set to final
    private static double kIntakeSpeed = 0.4;
    private static double kSlowSpeed = -0.1;

    private final CANSparkMax rollerMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kIntakeRoller);
    private final DoubleSolenoid intakePistons1 = new DoubleSolenoid(Constants.Drive.kIntakePistons[0], Constants.Drive.kIntakePistons[1]);
    private final DoubleSolenoid intakePistons2 = new DoubleSolenoid(Constants.Drive.kIntakePistons[2], Constants.Drive.kIntakePistons[3]);

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

}