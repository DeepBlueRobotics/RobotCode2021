package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeDownMotor = MotorControllerFactory.createSparkMax(Constants.Intake.kIntakeDownMotor);
    private final WPI_TalonSRX intakeMotor = MotorControllerFactory.createTalon(Constants.Intake.kIntakeMotor);
    public Intake(){
    }

    public void intake()
    {
        intakeMotor.set(Constants.Intake.kIntakeSpeed);
    }

    public void outtake()
    {
        intakeMotor.set(-Constants.Intake.kIntakeSpeed);
    }
    
    public void stop()
    {
        intakeMotor.set(0);
    }

    public void deploy() {
        intakeDownMotor.set(0.5); //TODO: set to correct value
    }

    public void retract() {
        intakeDownMotor.set(0.5); //TODO: set to correct value
    }

    public boolean isNotRunning() {
        return intakeMotor.get() == 0;
    }
}