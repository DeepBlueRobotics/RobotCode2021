package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intakeMotor = MotorControllerFactory.createTalon(Constants.Intake.INTAKE_MOTOR);
    private final DoubleSolenoid intakePistons = new DoubleSolenoid(Constants.Intake.INTAKE_PISTONS[0], Constants.Intake.INTAKE_PISTONS[1]);

    public Intake(){
    }

    public void intake()
    {
        intakeMotor.set(Constants.Intake.INTAKE_SPEED);
    }

    public void outtake()
    {
        intakeMotor.set(-Constants.Intake.INTAKE_SPEED);
    }
    
    public void stop()
    {
        intakeMotor.set(0);
    }

    public void deploy() {
        intakePistons.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        intakePistons.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isNotRunning() {
        return intakeMotor.get() == 0;
    }
}