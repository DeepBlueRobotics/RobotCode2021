package org.team199.robot2020.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static final double kIntakeSpeed = 1;

    private final CANSparkMax rollerMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kIntakeRoller);
    private final DoubleSolenoid intakePistons = new DoubleSolenoid(Constants.Drive.kIntakePistons[0], Constants.Drive.kIntakePistons[1]);

    public Intake(){
    }

    public void intake()
    {
        rollerMotor.set(kIntakeSpeed);
    }

    public void outtake()
    {
        rollerMotor.set(-kIntakeSpeed);
    }
    
    public void stop()
    {
        rollerMotor.set(0);
    }

    public void deploy() {
        intakePistons.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        intakePistons.set(DoubleSolenoid.Value.kReverse);
    }

    public void doTheFlop() {
        intakePistons.set(DoubleSolenoid.Value.kOff);
    }

    public boolean isNotRunning() {
        return rollerMotor.get() == 0;
    }
}